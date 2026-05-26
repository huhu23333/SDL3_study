#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>
#include <camera_projection.h>
#include <stb_image.h>

#include <cstdio>
#include <cmath>
#include <cstdlib>
#include <vector>
#include <algorithm>

#include <stb_image_write.h>

#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <opencv2/opencv.hpp>

// -----------------------------------------------------------------------------
// 摄像机姿态与坐标系变换
// -----------------------------------------------------------------------------
// 摄像机位于原点，初始朝向 +Z 轴。
// yaw：绕 Y 轴旋转，正值为摄像机向右看（鼠标右移）
// pitch：绕 X 轴旋转，正值为摄像机向上看（鼠标上移）
// roll：绕 Z 轴旋转（摄像机前方向），正值为顺时针旋转
struct CameraPose
{
    double yaw{0.0};
    double pitch{0.0};
    double roll{0.0};
};

// 将世界坐标系中的点变换到摄像机坐标系
// 先平移（减去摄像机位置），再旋转：R * (P_world - CamPos)
static Point3D WorldToCamera(const Point3D& world_pt, const Point3D& cam_pos, const CameraPose& cam)
{
    // 1) 平移
    double tx = world_pt.x - cam_pos.x;
    double ty = world_pt.y - cam_pos.y;
    double tz = world_pt.z - cam_pos.z;

    double cy = std::cos(cam.yaw);
    double sy = std::sin(cam.yaw);
    double cp = std::cos(cam.pitch);
    double sp = std::sin(cam.pitch);
    double cr = std::cos(cam.roll);
    double sr = std::sin(cam.roll);

    // 2) 绕 Y 轴旋转 -yaw
    double x1 =  tx * cy - tz * sy;
    double y1 =  ty;
    double z1 =  tx * sy + tz * cy;

    // 3) 绕 X 轴旋转 -pitch
    double x2 = x1;
    double y2 = y1 * cp + z1 * sp;
    double z2 = -y1 * sp + z1 * cp;

    // 4) 绕 Z 轴旋转 -roll（摄像机前方向轴）
    double x3 = x2 * cr - y2 * sr;
    double y3 = x2 * sr + y2 * cr;
    double z3 = z2;

    return Point3D{ x3, y3, z3 };
}

// -----------------------------------------------------------------------------
// 使用 stb_image 加载 PNG 并创建 SDL_Texture，同时返回纹理尺寸
// -----------------------------------------------------------------------------
struct TextureInfo {
    SDL_Texture* texture;
    int width;
    int height;
};

static TextureInfo LoadTextureFromPNG(SDL_Renderer* renderer, const char* filepath)
{
    TextureInfo info = { nullptr, 0, 0 };

    int width, height, channels;
    unsigned char* pixel_data = stbi_load(filepath, &width, &height, &channels, 4);
    if (!pixel_data) {
        SDL_Log("stbi_load failed for '%s': %s", filepath, stbi_failure_reason());
        return info;
    }

    SDL_Log("Loaded '%s': %dx%d, %d channels", filepath, width, height, channels);

    // OpenGL 下用 RGBA32（LE 上是 ABGR8888，字节序=R,G,B,A）
    // stb_image 正好输出 R,G,B,A，不需要任何转换
    info.texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGBA32,
                                     SDL_TEXTUREACCESS_STATIC, width, height);
    if (!info.texture) {
        SDL_Log("SDL_CreateTexture (RGBA32) failed: %s", SDL_GetError());
        stbi_image_free(pixel_data);
        return info;
    }

    SDL_UpdateTexture(info.texture, nullptr, pixel_data, width * 4);
    stbi_image_free(pixel_data);

    // OpenGL 下 NONE 混合应正常工作（不会像 Vulkan 那样偏蓝）
    // SDL_SetTextureBlendMode(info.texture, SDL_BLENDMODE_NONE);

    info.width = width;
    info.height = height;
    return info;
}

// -----------------------------------------------------------------------------
// 统一矩形面元系统：所有矩形（纹理面、边框面）按全局分块尺度细分
// -----------------------------------------------------------------------------
// 全局分块尺度（世界坐标单位/段）
// 每个矩形面将按此尺度划分为若干小四边形（每个四边形再分为 2 个三角形）
const double SUBDIV_SCALE = 0.05;

// 三维世界顶点（含 UV）
struct WorldVertex {
    Point3D pos;
    float u, v;
};

// 构建矩形面的三角形网格
// 将矩形 [cx±w/2, cy±h/2, cz] 划分为 SUBDIV_SCALE 大小的小面元
// uv_l,uv_t,uv_r,uv_b 为四角的 UV 坐标范围
// 输出到 out_verts（平三角形列表，每 3 个一组）
static void BuildFaceTriangles(std::vector<WorldVertex>& out_verts,
                               double cx, double cy, double cz,
                               double width, double height,
                               float uv_l, float uv_t,
                               float uv_r, float uv_b)
{
    int segs_w = std::max(1, (int)std::ceil(width / SUBDIV_SCALE));
    int segs_h = std::max(1, (int)std::ceil(height / SUBDIV_SCALE));
    double hw = width / 2.0, hh = height / 2.0;

    out_verts.clear();
    out_verts.reserve(segs_w * segs_h * 6);

    for (int gy = 0; gy < segs_h; ++gy) {
        for (int gx = 0; gx < segs_w; ++gx) {
            double txl = (double)gx / segs_w;
            double txr = (double)(gx + 1) / segs_w;
            double tyb = (double)gy / segs_h;
            double tyt = (double)(gy + 1) / segs_h;

            double xl = cx - hw + txl * width;
            double xr = cx - hw + txr * width;
            double yb = cy - hh + tyb * height;
            double yt = cy - hh + tyt * height;

            float ul = uv_l + (uv_r - uv_l) * (float)txl;
            float ur = uv_l + (uv_r - uv_l) * (float)txr;
            float vb = uv_t + (uv_b - uv_t) * (float)tyb;
            float vt = uv_t + (uv_b - uv_t) * (float)tyt;

            // 三角 1: 左下-右下-右上
            out_verts.push_back({ { xl, yb, cz }, ul, vb });
            out_verts.push_back({ { xr, yb, cz }, ur, vb });
            out_verts.push_back({ { xr, yt, cz }, ur, vt });
            // 三角 2: 左下-右上-左上
            out_verts.push_back({ { xl, yb, cz }, ul, vb });
            out_verts.push_back({ { xr, yt, cz }, ur, vt });
            out_verts.push_back({ { xl, yt, cz }, ul, vt });
        }
    }
}

// 渲染面：可以是纹理面或纯色面
struct RenderFace {
    std::vector<WorldVertex> world_verts;  // 3D 顶点列表
    SDL_Texture* texture;                   // nullptr = 纯色
    SDL_FColor color;                       // 纯色时的填充色
    bool apply_body_rotation = false;       // 是否应用本体坐标系旋转（最远的图像）
    double center_x = 0.0, center_y = 0.0, center_z = 0.0; // 旋转中心（世界坐标）
};

// -----------------------------------------------------------------------------
// 绕物体本体坐标系旋转
// 将点 point 绕中心 (cx, cy, cz) 旋转，旋转轴为物体的本体坐标轴：
//   yaw（绕本体 Y 轴，即向上轴）
//   pitch（绕本体 X 轴，即向右轴）
//   roll（绕本体 Z 轴，即向前轴）
// 旋转顺序：yaw -> pitch -> roll（本体坐标系内禀旋转）
// -----------------------------------------------------------------------------
static void ApplyBodyRotation(Point3D& point,
                              double cx, double cy, double cz,
                              double yaw, double pitch, double roll)
{
    // 平移到物体中心
    double dx = point.x - cx;
    double dy = point.y - cy;
    double dz = point.z - cz;

    double cy_a = std::cos(yaw);
    double sy_a = std::sin(yaw);
    double cp_a = std::cos(pitch);
    double sp_a = std::sin(pitch);
    double cr_a = std::cos(roll);
    double sr_a = std::sin(roll);

    // 内禀旋转 R = R_y(yaw) * R_x(pitch) * R_z(roll)
    // 先绕本体 Z 轴旋转（roll）
    double x1 = dx * cr_a - dy * sr_a;
    double y1 = dx * sr_a + dy * cr_a;
    double z1 = dz;

    // 再绕本体 X 轴旋转（pitch）
    double x2 = x1;
    double y2 = y1 * cp_a + z1 * sp_a;
    double z2 = -y1 * sp_a + z1 * cp_a;

    // 最后绕本体 Y 轴旋转（yaw）
    double x3 = x2 * cy_a + z2 * sy_a;
    double y3 = y2;
    double z3 = -x2 * sy_a + z2 * cy_a;

    // 平移回原位置
    point.x = x3 + cx;
    point.y = y3 + cy;
    point.z = z3 + cz;
}

// -----------------------------------------------------------------------------
// 关键点数据结构
// -----------------------------------------------------------------------------
struct Keypoint
{
    int index;
    double pixel_x;  // 像素坐标 x（在 target.png 中）
    double pixel_y;  // 像素坐标 y（在 target.png 中）
};

// -----------------------------------------------------------------------------
// 使用 OpenCV 渲染文字到带透明通道的 SDL_Texture
// -----------------------------------------------------------------------------
static SDL_Texture* RenderTextToTexture(SDL_Renderer* renderer, const char* text,
                                          SDL_FColor color, double font_scale = 0.5,
                                          int thickness = 1)
{
    int baseline = 0;
    cv::Size text_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX,
                                          font_scale, thickness, &baseline);
    if (text_size.width <= 0 || text_size.height <= 0) return nullptr;

    int pad = 2;
    int w = text_size.width + pad * 2;
    int h = text_size.height + baseline + pad * 2;

    // 4 通道 RGBA 图像，初始化为全透明
    cv::Mat img(h, w, CV_8UC4, cv::Scalar(0, 0, 0, 0));

    // 绘制白色文字（在 SDL 纹理上通过 color 调节色调）
    cv::putText(img, text, cv::Point(pad, h - pad - baseline),
                cv::FONT_HERSHEY_SIMPLEX, font_scale,
                cv::Scalar(255, 255, 255, 255), thickness, cv::LINE_AA);

    // 创建 SDL 纹理
    SDL_Texture* texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGBA32,
                                             SDL_TEXTUREACCESS_STATIC, w, h);
    if (!texture) {
        SDL_Log("SDL_CreateTexture failed in RenderTextToTexture: %s", SDL_GetError());
        return nullptr;
    }
    SDL_UpdateTexture(texture, nullptr, img.data, w * 4);
    SDL_SetTextureBlendMode(texture, SDL_BLENDMODE_BLEND);

    return texture;
}

// 在渲染器上绘制一个填充圆（使用三角形扇形）
static void DrawFilledCircle(SDL_Renderer* renderer, float cx, float cy, float radius, SDL_FColor color)
{
    const int SEGMENTS = 24;
    std::vector<SDL_Vertex> verts;
    verts.reserve((SEGMENTS + 1) * 3);

    // 圆心
    SDL_Vertex center = { { cx, cy }, color, { 0.0f, 0.0f } };

    for (int i = 0; i < SEGMENTS; ++i) {
        float a1 = (float)(2.0 * M_PI * i / SEGMENTS);
        float a2 = (float)(2.0 * M_PI * (i + 1) / SEGMENTS);

        float x1 = cx + radius * std::cos(a1);
        float y1 = cy + radius * std::sin(a1);
        float x2 = cx + radius * std::cos(a2);
        float y2 = cy + radius * std::sin(a2);

        verts.push_back(center);
        verts.push_back({ { x1, y1 }, color, { 0.0f, 0.0f } });
        verts.push_back({ { x2, y2 }, color, { 0.0f, 0.0f } });
    }

    SDL_RenderGeometry(renderer, nullptr, verts.data(), (int)verts.size(), nullptr, 0);
}

// -----------------------------------------------------------------------------
// 目标图像面的相关参数（用于关键点坐标映射）
// -----------------------------------------------------------------------------
struct TargetFaceInfo {
    double center_x, center_y, center_z;
    double half_width, half_height;
    double width, height;
};

// 从关键点像素坐标计算其世界坐标（在目标面上，未旋转前）
static Point3D KeypointPixelToWorld(const Keypoint& kp, const TargetFaceInfo& face,
                                     int tex_w, int tex_h)
{
    double u = kp.pixel_x / tex_w;   // 0=left, 1=right
    double v = kp.pixel_y / tex_h;   // 0=top, 1=bottom
    // 世界坐标映射：UV (0,0)=top-left of image → (cx-hw, cy+hh) top in world
    // 但由于世界坐标系中 y 朝下为正，而 UV v 向下递增，两者方向一致
    double wx = face.center_x - face.half_width  + u * face.width;
    double wy = face.center_y - face.half_height + v * face.height;
    double wz = face.center_z;
    return { wx, wy, wz };
}

// -----------------------------------------------------------------------------
// 主函数
// -----------------------------------------------------------------------------
int main(int argc, char* argv[])
{
    // ---------- 1. SDL 初始化 ----------
    SDL_SetLogPriorities(SDL_LOG_PRIORITY_VERBOSE);

    if (!SDL_Init(SDL_INIT_VIDEO)) {
        SDL_Log("SDL_Init failed: %s", SDL_GetError());
        return -1;
    }

    const int LOGICAL_WIDTH = 3840;
    const int LOGICAL_HEIGHT = 2160;
    const int WINDOW_WIDTH = 1920;
    const int WINDOW_HEIGHT = 1080;

    SDL_Window* window = SDL_CreateWindow(
        "Camera Peek - Mouse Look around Fixed Image",
        WINDOW_WIDTH, WINDOW_HEIGHT,
        SDL_WINDOW_RESIZABLE);
    if (!window) {
        SDL_Log("Could not create a window: %s", SDL_GetError());
        return -1;
    }
    SDL_SetWindowPosition(window, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED);

    SDL_Renderer* renderer = SDL_CreateRenderer(window, "opengl");
    if (!renderer) {
        SDL_Log("Create renderer failed: %s", SDL_GetError());
        return -1;
    }
    SDL_SetRenderVSync(renderer, 0);

    // 创建离屏渲染目标纹理，始终渲染到透明背景
    // 纹理为预乘alpha格式，用于显示叠加和透明度截图
    SDL_Texture* offscreen = SDL_CreateTexture(
        renderer, SDL_PIXELFORMAT_RGBA32,
        SDL_TEXTUREACCESS_TARGET, LOGICAL_WIDTH, LOGICAL_HEIGHT);
    if (!offscreen) {
        SDL_Log("Create offscreen texture failed: %s", SDL_GetError());
    }
    // 当离屏纹理作为源渲染到背景上时，使用预乘alpha混合
    SDL_SetTextureBlendMode(offscreen, SDL_BLENDMODE_BLEND_PREMULTIPLIED);

    // ---------- 2. 加载纹理 ----------
    TextureInfo tex_info = LoadTextureFromPNG(renderer, "images/demo.png");
    if (!tex_info.texture) {
        SDL_Log("Failed to load texture.");
        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);
        SDL_Quit();
        return -1;
    }
    SDL_Log("Texture size: %dx%d", tex_info.width, tex_info.height);

    // ---------- 2b. 加载 target 纹理 ----------
    TextureInfo target_tex_info = LoadTextureFromPNG(renderer, "images/target.png");
    if (!target_tex_info.texture) {
        SDL_Log("Failed to load target texture.");
        SDL_DestroyTexture(tex_info.texture);
        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);
        SDL_Quit();
        return -1;
    }
    SDL_Log("Target texture size: %dx%d", target_tex_info.width, target_tex_info.height);

    // ---------- 2c. 读取关键点文件 ----------
    std::vector<Keypoint> keypoints;
    {
        std::ifstream kf("images/target.txt");
        if (!kf.is_open()) {
            SDL_Log("Warning: Could not open images/target.txt");
        } else {
            std::string line;
            while (std::getline(kf, line)) {
                if (line.empty()) continue;
                std::istringstream iss(line);
                Keypoint kp;
                if (iss >> kp.index >> kp.pixel_x >> kp.pixel_y) {
                    keypoints.push_back(kp);
                }
            }
            SDL_Log("Loaded %zu keypoints from target.txt", keypoints.size());
        }
    }

    // ---------- 3. 设置相机参数 ----------
    CameraIntrinsics intrinsics{
        960, 960,
        LOGICAL_WIDTH / 2.0, LOGICAL_HEIGHT / 2.0,
        LOGICAL_WIDTH, LOGICAL_HEIGHT   // 启用画面最大半视场角检查
    };
    DistortionCoefficients distortion{0.0, 0.0, 0.0, 0.0, 0.0};

    // 计算并输出画面最大半视场角（考虑了畸变系数后）
    double max_half_fov = ComputeMaxHalfFovAngle(intrinsics, distortion);
    SDL_Log("Max half FOV angle (w/ distortion): %.2f deg",
            max_half_fov * 180.0 / M_PI);
    double diagonal_fov = 2.0 * std::atan(
        std::sqrt((double)(LOGICAL_WIDTH * LOGICAL_WIDTH +
                           LOGICAL_HEIGHT * LOGICAL_HEIGHT)) * 0.5 / 960.0);
    SDL_Log("Diagonal FOV: %.2f deg",
            diagonal_fov * 180.0 / M_PI);

    // ---------- 4. 构建所有渲染面（纹理面 + 边框面） ----------
    double base_height = 2.0;
    double base_width = base_height * (double)tex_info.width / (double)tex_info.height;
    double rect_depth = 2.0;
    double hw = base_width / 2.0, hh = base_height / 2.0;
    double border_w = 0.04;  // 边框厚度（世界坐标单位）

    SDL_Log("Rect aspect ratio: %.2f (image: %dx%d), border: %.3f units",
            base_width / base_height, tex_info.width, tex_info.height, border_w);

    // 收集所有面
    std::vector<RenderFace> all_faces;
    SDL_FColor border_color = { 1.0f, 1.0f, 0.39f, 1.0f };

    // 面 0: 后方纹理面
    all_faces.emplace_back();
    BuildFaceTriangles(all_faces.back().world_verts,
                       0, 0, rect_depth + 3.0,
                       base_width, base_height,
                       0.0f, 0.0f, 1.0f, 1.0f);
    all_faces.back().texture = tex_info.texture;
    all_faces.back().color = { 1.0f, 1.0f, 1.0f, 0.7f };

    // // 面 10：最远的纹理面（可绕本体坐标系旋转）
    // all_faces.emplace_back();
    // BuildFaceTriangles(all_faces.back().world_verts,
    //                    0, 0, rect_depth + 6.0,
    //                    base_width, base_height,
    //                    0.0f, 0.0f, 1.0f, 1.0f);
    // all_faces.back().texture = tex_info.texture;
    // all_faces.back().color = { 1.0f, 1.0f, 1.0f, 1.0f };
    // all_faces.back().apply_body_rotation = true;
    // all_faces.back().center_x = 0.0;
    // all_faces.back().center_y = 0.0;
    // all_faces.back().center_z = rect_depth + 6.0;

    // 面 11（target 图像面）：在面 10 后方，和面 10 一样转动
    double target_base_height = 5.0;
    double target_base_width = target_base_height * (double)target_tex_info.width / (double)target_tex_info.height;
    double target_z = rect_depth + 9.0;
    double target_hw = target_base_width / 2.0;
    double target_hh = target_base_height / 2.0;

    TargetFaceInfo target_face_info = { 0.0, 0.0, target_z, target_hw, target_hh,
                                        target_base_width, target_base_height };

    all_faces.emplace_back();
    BuildFaceTriangles(all_faces.back().world_verts,
                       0, 0, target_z,
                       target_base_width, target_base_height,
                       0.0f, 0.0f, 1.0f, 1.0f);
    all_faces.back().texture = target_tex_info.texture;
    all_faces.back().color = { 1.0f, 1.0f, 1.0f, 1.0f };
    all_faces.back().apply_body_rotation = true;
    all_faces.back().center_x = 0.0;
    all_faces.back().center_y = 0.0;
    all_faces.back().center_z = target_z;

    // 预渲染关键点序号纹理（使用 OpenCV）
    std::vector<SDL_Texture*> index_textures(keypoints.size(), nullptr);
    for (size_t i = 0; i < keypoints.size(); ++i) {
        char idx_str[16];
        std::snprintf(idx_str, sizeof(idx_str), "%d", keypoints[i].index);
        index_textures[i] = RenderTextToTexture(renderer, idx_str,
                                                  { 0.0f, 1.0f, 0.0f, 1.0f },
                                                  0.8, 2);
    }

    bool show_keypoints = false;

    // 面 1: 前方纹理面
    all_faces.emplace_back();
    BuildFaceTriangles(all_faces.back().world_verts,
                       0, 0, rect_depth,
                       base_width, base_height,
                       0.0f, 0.0f, 1.0f, 1.0f);
    all_faces.back().texture = tex_info.texture;
    all_faces.back().color = { 1,1,1,1 };

    // 面 1-4: 边框（4 条薄矩形条，在 3D 中围绕纹理面）
    // 左条
    all_faces.emplace_back();
    BuildFaceTriangles(all_faces.back().world_verts,
                       -(hw + border_w / 2.0), 0, rect_depth,
                       border_w, base_height,
                       0, 0, 0, 0);
    all_faces.back().texture = nullptr;
    all_faces.back().color = border_color;

    // 右条
    all_faces.emplace_back();
    BuildFaceTriangles(all_faces.back().world_verts,
                       (hw + border_w / 2.0), 0, rect_depth,
                       border_w, base_height,
                       0, 0, 0, 0);
    all_faces.back().texture = nullptr;
    all_faces.back().color = border_color;

    // 上条（含左右两侧至边框外沿）
    all_faces.emplace_back();
    BuildFaceTriangles(all_faces.back().world_verts,
                       0, (hh + border_w / 2.0), rect_depth,
                       base_width + 2.0 * border_w, border_w,
                       0, 0, 0, 0);
    all_faces.back().texture = nullptr;
    all_faces.back().color = border_color;

    // 下条
    all_faces.emplace_back();
    BuildFaceTriangles(all_faces.back().world_verts,
                       0, -(hh + border_w / 2.0), rect_depth,
                       base_width + 2.0 * border_w, border_w,
                       0, 0, 0, 0);
    all_faces.back().texture = nullptr;
    all_faces.back().color = border_color;

    // ---------- 5. 鼠标/键盘视角与移动控制 ----------
    bool mouse_grabbed = false;
    CameraPose camera{};
    Point3D cam_pos{ 0.0, 0.0, 0.0 };  // 摄像机在世界坐标系中的位置
    float mouse_sensitivity = 0.002f;
    float move_speed = 2.0f;  // 单位/秒

    // ---------- 最远图像的物体本体旋转控制 ----------
    double obj_yaw = 0.0;    // 绕本体 Y 轴（向上轴）
    double obj_pitch = 0.0;  // 绕本体 X 轴（向右轴）
    double obj_roll = 0.0;   // 绕本体 Z 轴（向前轴）
    float obj_rot_speed = 2.0f;  // 弧度/秒

    // 键盘状态
    bool key_w = false, key_s = false, key_a = false, key_d = false;
    bool key_up = false, key_down = false;
    bool key_q = false, key_e = false;
    bool key_j = false, key_l = false;  // yaw（本体 Y 轴）
    bool key_i = false, key_k = false;  // pitch（本体 X 轴）
    bool key_u = false, key_o = false;  // roll（本体 Z 轴）
    float roll_speed = 1.5f;  // 弧度/秒

    bool screenshot_requested = false;

    // 缓存关键点坐标纹理（避免每帧创建）
    // glo=全局世界坐标, cam=相机空间坐标, pix=渲染画面像素坐标
    std::vector<SDL_Texture*> glo_textures(keypoints.size(), nullptr);
    std::vector<SDL_Texture*> cam_textures(keypoints.size(), nullptr);
    std::vector<SDL_Texture*> pix_textures(keypoints.size(), nullptr);
    std::vector<std::string> cached_glo_texts(keypoints.size());
    std::vector<std::string> cached_cam_texts(keypoints.size());
    std::vector<std::string> cached_pix_texts(keypoints.size());

    // ---------- 6. 主循环 ----------
    SDL_Event event{};
    bool keep_going = true;
    int frame_count = 0;
    uint64_t fps_last_ticks = SDL_GetTicks();

    SDL_Log("Click inside the window to capture mouse. Move mouse to look around.");
    SDL_Log("WASD: move on water plane | SPACE: up | SHIFT: down | ESC: release/quit");
    SDL_Log("Q/E: roll camera | R: reset roll");
    SDL_Log("J/L: yaw far image (body Y/up) | I/K: pitch (body X/right) | U/O: roll (body Z/forward)");
    SDL_Log("M: toggle keypoint annotations on the target image");

    uint64_t prev_ticks = SDL_GetTicks();

    while (keep_going) {
        ++frame_count;

        uint64_t current_ticks = SDL_GetTicks();
        float dt = (current_ticks - prev_ticks) / 1000.0f;
        prev_ticks = current_ticks;
        if (dt > 0.05f) dt = 0.05f;  // 防止大跳

        // FPS
        if (current_ticks - fps_last_ticks >= 1000) {
            SDL_Log("FPS: %d", frame_count);
            frame_count = 0;
            fps_last_ticks = current_ticks;
        }

        // 事件处理
        while (SDL_PollEvent(&event)) {
            switch (event.type) {
            case SDL_EVENT_QUIT:
                keep_going = false;
                break;

            case SDL_EVENT_KEY_DOWN:
                switch (event.key.key) {
                case SDLK_ESCAPE:
                    if (mouse_grabbed) {
                        SDL_SetWindowRelativeMouseMode(window, false);
                        mouse_grabbed = false;
                        SDL_Log("Mouse released. Click to capture again.");
                    } else {
                        keep_going = false;
                    }
                    break;
                case SDLK_W: key_w = true; break;
                case SDLK_S: key_s = true; break;
                case SDLK_A: key_a = true; break;
                case SDLK_D: key_d = true; break;
                case SDLK_SPACE: key_up = true; break;
                case SDLK_LSHIFT: case SDLK_RSHIFT: key_down = true; break;
                case SDLK_Q: key_q = true; break;
                case SDLK_E: key_e = true; break;
                case SDLK_R: camera.roll = 0.0; break;
                case SDLK_J: key_j = true; break;
                case SDLK_L: key_l = true; break;
                case SDLK_I: key_i = true; break;
                case SDLK_K: key_k = true; break;
                case SDLK_U: key_u = true; break;
                case SDLK_O: key_o = true; break;
                case SDLK_M:
                    if (event.key.repeat == 0) {
                        show_keypoints = !show_keypoints;
                        SDL_Log("Keypoint display: %s", show_keypoints ? "ON" : "OFF");
                    }
                    break;
                case SDLK_P:
                    if (event.key.repeat == 0) {
                        screenshot_requested = true;
                    }
                    break;
                default: break;
                }
                break;

            case SDL_EVENT_KEY_UP:
                switch (event.key.key) {
                case SDLK_W: key_w = false; break;
                case SDLK_S: key_s = false; break;
                case SDLK_A: key_a = false; break;
                case SDLK_D: key_d = false; break;
                case SDLK_SPACE: key_up = false; break;
                case SDLK_LSHIFT: case SDLK_RSHIFT: key_down = false; break;
                case SDLK_Q: key_q = false; break;
                case SDLK_E: key_e = false; break;
                case SDLK_J: key_j = false; break;
                case SDLK_L: key_l = false; break;
                case SDLK_I: key_i = false; break;
                case SDLK_K: key_k = false; break;
                case SDLK_U: key_u = false; break;
                case SDLK_O: key_o = false; break;
                default: break;
                }
                break;

            case SDL_EVENT_MOUSE_BUTTON_DOWN:
                if (!mouse_grabbed) {
                    SDL_SetWindowRelativeMouseMode(window, true);
                    mouse_grabbed = true;
                    SDL_Log("Mouse captured. Move mouse to look around.");
                }
                break;

            case SDL_EVENT_MOUSE_MOTION:
                if (mouse_grabbed) {
                    camera.yaw   += event.motion.xrel * mouse_sensitivity;
                    camera.pitch -= event.motion.yrel * mouse_sensitivity;
                    const double pitch_limit = 1.5; // ~86°
                    if (camera.pitch >  pitch_limit) camera.pitch =  pitch_limit;
                    if (camera.pitch < -pitch_limit) camera.pitch = -pitch_limit;
                }
                break;

            case SDL_EVENT_WINDOW_RESIZED: {
                // 逻辑分辨率固定 1920x1080，主点不变
                break;
            }
            }
        }

        // ---------- 摄像机移动（水面内移动） ----------
        // 计算摄像机朝向向量（世界空间）
        double cy = std::cos(camera.yaw);
        double sy = std::sin(camera.yaw);
        double cp = std::cos(camera.pitch);
        double sp = std::sin(camera.pitch);
        // 摄像机在世界空间中的三个轴
        // 前: R^T * (0,0,1) = (cp*sy, -sp, cp*cy)
        // 右: R^T * (1,0,0) = (cy, 0, -sy)
        // 上（世界 Y 轴负方向）
        Point3D forward{ cp * sy, -sp, cp * cy };
        Point3D right{ cy, 0.0, -sy };

        // 水面内移动：将前方向投影到水平面（XZ 平面），忽略 pitch 的 y 分量
        // 水面内的前方向 = normalize(sy, 0, cy)，右方向 = (cy, 0, -sy)
        double wf_x = sy;
        double wf_z = cy;
        double wr_x = cy;
        double wr_z = -sy;

        if (key_w) { cam_pos.x += wf_x * move_speed * dt; cam_pos.z += wf_z * move_speed * dt; }
        if (key_s) { cam_pos.x -= wf_x * move_speed * dt; cam_pos.z -= wf_z * move_speed * dt; }
        if (key_a) { cam_pos.x -= wr_x * move_speed * dt; cam_pos.z -= wr_z * move_speed * dt; }
        if (key_d) { cam_pos.x += wr_x * move_speed * dt; cam_pos.z += wr_z * move_speed * dt; }
        if (key_up)   cam_pos.y -= move_speed * dt;  // 世界 Y 向下，减 = 向上
        if (key_down) cam_pos.y += move_speed * dt;  // 世界 Y 向下，加 = 向下

        // Roll 控制
        if (key_q) camera.roll += roll_speed * dt;
        if (key_e) camera.roll -= roll_speed * dt;

        // ---------- 最远图像的物体本体旋转控制 ----------
        if (key_j) obj_yaw   += obj_rot_speed * dt;   // 绕本体 Y 轴（向上轴）正转
        if (key_l) obj_yaw   -= obj_rot_speed * dt;   // 绕本体 Y 轴（向上轴）反转
        if (key_i) obj_pitch += obj_rot_speed * dt;   // 绕本体 X 轴（向右轴）正转
        if (key_k) obj_pitch -= obj_rot_speed * dt;   // 绕本体 X 轴（向右轴）反转
        if (key_u) obj_roll  += obj_rot_speed * dt;   // 绕本体 Z 轴（向前轴）正转
        if (key_o) obj_roll  -= obj_rot_speed * dt;   // 绕本体 Z 轴（向前轴）反转

        // ---------- 渲染到离屏纹理（固定分辨率，透明背景） ----------
        SDL_SetRenderTarget(renderer, offscreen);

        const float MAX_COORD = 1e6f;
        // 清为全透明黑色 → 未渲染区域为全透明
        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 0);
        SDL_RenderClear(renderer);

        // 收集所有可见面及其在摄像机空间中的平均深度
        struct SortedFace {
            const RenderFace* face;
            std::vector<SDL_Vertex> sdl_verts;
            double cam_z;  // 摄像机空间深度（越大越远）
        };
        std::vector<SortedFace> sorted_faces;
        sorted_faces.reserve(all_faces.size());

        for (const auto& face : all_faces) {
            SortedFace sf;
            sf.face = &face;
            sf.sdl_verts.reserve(face.world_verts.size());
            double z_sum = 0.0;
            int visible_tris = 0;

            for (size_t i = 0; i < face.world_verts.size(); i += 3) {
                const WorldVertex& wv0 = face.world_verts[i];
                const WorldVertex& wv1 = face.world_verts[i + 1];
                const WorldVertex& wv2 = face.world_verts[i + 2];

                // 如果此面标记了本体旋转，则先对顶点应用物体本体坐标系旋转
                Point3D r0 = wv0.pos;
                Point3D r1 = wv1.pos;
                Point3D r2 = wv2.pos;

                if (face.apply_body_rotation) {
                    ApplyBodyRotation(r0, face.center_x, face.center_y, face.center_z,
                                      obj_yaw, obj_pitch, obj_roll);
                    ApplyBodyRotation(r1, face.center_x, face.center_y, face.center_z,
                                      obj_yaw, obj_pitch, obj_roll);
                    ApplyBodyRotation(r2, face.center_x, face.center_y, face.center_z,
                                      obj_yaw, obj_pitch, obj_roll);
                }

                Point3D c0 = WorldToCamera(r0, cam_pos, camera);
                Point3D c1 = WorldToCamera(r1, cam_pos, camera);
                Point3D c2 = WorldToCamera(r2, cam_pos, camera);

                Point2D pp0 = ProjectPoint(c0, intrinsics, distortion);
                Point2D pp1 = ProjectPoint(c1, intrinsics, distortion);
                Point2D pp2 = ProjectPoint(c2, intrinsics, distortion);

                // 只要至少有一个顶点有效就渲染该三角形（GPU 自动裁剪无效区域）
                if (!pp0.valid && !pp1.valid && !pp2.valid)
                    continue;
                if (std::isnan(pp0.x) || std::isnan(pp0.y) ||
                    std::isnan(pp1.x) || std::isnan(pp1.y) ||
                    std::isnan(pp2.x) || std::isnan(pp2.y))
                    continue;
                if (std::abs(pp0.x) > MAX_COORD || std::abs(pp0.y) > MAX_COORD ||
                    std::abs(pp1.x) > MAX_COORD || std::abs(pp1.y) > MAX_COORD ||
                    std::abs(pp2.x) > MAX_COORD || std::abs(pp2.y) > MAX_COORD)
                    continue;

                sf.sdl_verts.push_back({ { (float)pp0.x, (float)pp0.y }, face.color, { wv0.u, wv0.v } });
                sf.sdl_verts.push_back({ { (float)pp1.x, (float)pp1.y }, face.color, { wv1.u, wv1.v } });
                sf.sdl_verts.push_back({ { (float)pp2.x, (float)pp2.y }, face.color, { wv2.u, wv2.v } });
                z_sum += c0.z + c1.z + c2.z;
                visible_tris += 3;
            }

            if (visible_tris > 0) {
                sf.cam_z = z_sum / visible_tris;
                sorted_faces.push_back(std::move(sf));
            }
        }

        // 按深度从远到近排序（cam_z 越大越远）
        std::sort(sorted_faces.begin(), sorted_faces.end(),
            [](const SortedFace& a, const SortedFace& b) { return a.cam_z > b.cam_z; });

        // 渲染
        for (const auto& sf : sorted_faces) {
            SDL_RenderGeometry(renderer, sf.face->texture,
                               sf.sdl_verts.data(), (int)sf.sdl_verts.size(),
                               nullptr, 0);
        }

        SDL_FlushRenderer(renderer);

        // 绿色十字丝（主点）—— 始终在 2D 屏幕空间绘制
        {
            SDL_FColor green = { 0.39f, 1.0f, 0.39f, 1.0f };
            float cx = (float)intrinsics.cx;
            float cy = (float)intrinsics.cy;
            float hw = 20.0f;
            float thick = 4.0f;

            SDL_Vertex h_verts[6] = {
                { { cx - hw, cy - thick }, green, { 0, 0 } },
                { { cx + hw, cy - thick }, green, { 0, 0 } },
                { { cx - hw, cy + thick }, green, { 0, 0 } },
                { { cx + hw, cy - thick }, green, { 0, 0 } },
                { { cx + hw, cy + thick }, green, { 0, 0 } },
                { { cx - hw, cy + thick }, green, { 0, 0 } }
            };
            SDL_RenderGeometry(renderer, nullptr, h_verts, 6, nullptr, 0);

            SDL_Vertex v_verts[6] = {
                { { cx - thick, cy - hw }, green, { 0, 0 } },
                { { cx + thick, cy - hw }, green, { 0, 0 } },
                { { cx - thick, cy + hw }, green, { 0, 0 } },
                { { cx + thick, cy - hw }, green, { 0, 0 } },
                { { cx + thick, cy + hw }, green, { 0, 0 } },
                { { cx - thick, cy + hw }, green, { 0, 0 } }
            };
            SDL_RenderGeometry(renderer, nullptr, v_verts, 6, nullptr, 0);
        }

        // ------ 关键点渲染（当 show_keypoints 开启时） ------
        if (show_keypoints && !keypoints.empty()) {
            SDL_FColor kp_color_dot = { 0.0f, 1.0f, 0.0f, 1.0f };    // 绿色圆点
            SDL_FColor kp_color_idx = { 0.0f, 1.0f, 0.0f, 1.0f };    // 绿色序号
            SDL_FColor kp_color_pos = { 1.0f, 1.0f, 0.4f, 1.0f };    // 黄色坐标

            for (size_t ki = 0; ki < keypoints.size(); ++ki) {
                // 1) 像素坐标 → 世界坐标（未旋转）
                Point3D world_pt = KeypointPixelToWorld(keypoints[ki], target_face_info,
                                                         target_tex_info.width, target_tex_info.height);

                // 2) 应用本体旋转（与面 10、面 11 相同）
                ApplyBodyRotation(world_pt, target_face_info.center_x,
                                  target_face_info.center_y, target_face_info.center_z,
                                  obj_yaw, obj_pitch, obj_roll);

                // 3) 变换到相机空间 → 投影到屏幕
                Point3D cam_pt = WorldToCamera(world_pt, cam_pos, camera);
                if (cam_pt.z <= 0.001) continue;

                Point2D screen_pt = ProjectPoint(cam_pt, intrinsics, distortion);
                if (std::isnan(screen_pt.x) || std::isnan(screen_pt.y) ||
                    std::abs(screen_pt.x) > MAX_COORD || std::abs(screen_pt.y) > MAX_COORD)
                    continue;

                float sx = (float)screen_pt.x;
                float sy = (float)screen_pt.y;

                // 4) 绘制关键点圆点
                DrawFilledCircle(renderer, sx, sy, 10.0f, kp_color_dot);

                // 5) 绘制序号纹理（预渲染好的）
                if (ki < index_textures.size() && index_textures[ki]) {
                    float tw, th;
                    SDL_GetTextureSize(index_textures[ki], &tw, &th);
                    SDL_FRect dst = { sx + 10.0f, sy - th / 2.0f,
                                      tw, th };
                    SDL_RenderTexture(renderer, index_textures[ki], nullptr, &dst);
                }

                // 6) 绘制 glo/cam/pix 三行坐标文字
                char glo_str[64], cam_str[64], pix_str[64];
                std::snprintf(glo_str, sizeof(glo_str), "glo:(%.2f, %.2f, %.2f)",
                             world_pt.x, world_pt.y, world_pt.z);
                std::snprintf(cam_str, sizeof(cam_str), "cam:(%.2f, %.2f, %.2f)",
                             cam_pt.x, cam_pt.y, cam_pt.z);
                std::snprintf(pix_str, sizeof(pix_str), "pix:(%.1f, %.1f)",
                             screen_pt.x, screen_pt.y);

                float line_y = sy + 10.0f;

                // glo — 浅绿色
                if (ki < cached_glo_texts.size()) {
                    if (cached_glo_texts[ki] != glo_str) {
                        cached_glo_texts[ki] = glo_str;
                        if (glo_textures[ki]) SDL_DestroyTexture(glo_textures[ki]);
                        glo_textures[ki] = RenderTextToTexture(renderer, glo_str,
                                                                 { 0.4f, 1.0f, 0.4f, 1.0f }, 0.55, 2);
                    }
                    if (glo_textures[ki]) {
                        float tw, th;
                        SDL_GetTextureSize(glo_textures[ki], &tw, &th);
                        SDL_FRect dst = { sx + 14.0f, line_y, tw, th };
                        SDL_RenderTexture(renderer, glo_textures[ki], nullptr, &dst);
                        line_y += th + 2.0f;
                    }
                }

                // cam — 天蓝色
                if (ki < cached_cam_texts.size()) {
                    if (cached_cam_texts[ki] != cam_str) {
                        cached_cam_texts[ki] = cam_str;
                        if (cam_textures[ki]) SDL_DestroyTexture(cam_textures[ki]);
                        cam_textures[ki] = RenderTextToTexture(renderer, cam_str,
                                                                 { 0.4f, 0.8f, 1.0f, 1.0f }, 0.55, 2);
                    }
                    if (cam_textures[ki]) {
                        float tw, th;
                        SDL_GetTextureSize(cam_textures[ki], &tw, &th);
                        SDL_FRect dst = { sx + 14.0f, line_y, tw, th };
                        SDL_RenderTexture(renderer, cam_textures[ki], nullptr, &dst);
                        line_y += th + 2.0f;
                    }
                }

                // pix — 黄色
                if (ki < cached_pix_texts.size()) {
                    if (cached_pix_texts[ki] != pix_str) {
                        cached_pix_texts[ki] = pix_str;
                        if (pix_textures[ki]) SDL_DestroyTexture(pix_textures[ki]);
                        pix_textures[ki] = RenderTextToTexture(renderer, pix_str,
                                                                 { 1.0f, 1.0f, 0.4f, 1.0f }, 0.55, 2);
                    }
                    if (pix_textures[ki]) {
                        float tw, th;
                        SDL_GetTextureSize(pix_textures[ki], &tw, &th);
                        SDL_FRect dst = { sx + 14.0f, line_y, tw, th };
                        SDL_RenderTexture(renderer, pix_textures[ki], nullptr, &dst);
                    }
                }
            }
        }

        // 显示提示（只有当鼠标未捕获时）
        if (!mouse_grabbed) {
            SDL_SetRenderDrawColor(renderer, 200, 200, 200, 180);
            // 简单画个文字提示太复杂，略过
        }

        // ----- 截图：从离屏纹理读像素（3840x2160，含 alpha 通道） -----
        if (screenshot_requested) {
            screenshot_requested = false;

            SDL_Surface* frame_surface =
                SDL_RenderReadPixels(renderer, nullptr);
            if (frame_surface) {
                // 转为 RGBA32（LE 上 R,G,B,A 内存字节序，与 stb 一致）
                SDL_Surface* converted = SDL_ConvertSurface(
                    frame_surface, SDL_PIXELFORMAT_RGBA32);
                if (converted) {
                    int capture_w = converted->w;
                    int capture_h = converted->h;
                    size_t pitch = static_cast<size_t>(capture_w) * 4;

                    // 拷贝像素到内存数组
                    std::vector<uint8_t> rgba_pixels(
                        static_cast<size_t>(capture_h) * pitch);
                    memcpy(rgba_pixels.data(),
                           converted->pixels,
                           static_cast<size_t>(capture_h) * pitch);

                    // 还原非预乘 alpha（SDL 渲染产生的是预乘 alpha）
                    // 未渲染区域: (0,0,0,0) → 保持 (0,0,0,0) 全透明
                    // 半透明物体: (R*a, G*a, B*a, a) → (R, G, B, a)
                    // 不透明物体: (R, G, B, 255) → 不变
                    for (size_t i = 0; i < static_cast<size_t>(capture_w) * capture_h; ++i) {
                        uint8_t* p = rgba_pixels.data() + i * 4;
                        uint8_t a = p[3];
                        if (a > 0 && a < 255) {
                            p[0] = (uint8_t)(((uint16_t)p[0] * 255) / a);
                            p[1] = (uint8_t)(((uint16_t)p[1] * 255) / a);
                            p[2] = (uint8_t)(((uint16_t)p[2] * 255) / a);
                        }
                        // a == 0: 像素已为 (0,0,0,0)，无需处理
                        // a == 255: 颜色已是原始值，无需处理
                    }

                    // ✓ rgba_pixels 内存数组已提取完毕
                    //   - 无渲染区域: (0,0,0,0) 全透明
                    //   - 半透明区域: 保留了多层叠加的 alpha
                    //   - 不透明区域: (R,G,B,255)

                    static int screenshot_counter = 0;
                    char filename[64];
                    std::snprintf(filename, sizeof(filename),
                        "screenshot_%03d.png",
                        screenshot_counter++);

                    // 存为带透明度的 PNG
                    if (stbi_write_png(filename, capture_w, capture_h,
                            4, rgba_pixels.data(),
                            static_cast<int>(pitch))) {
                        SDL_Log("Screenshot saved: %s (%dx%d)",
                            filename, capture_w, capture_h);
                    } else {
                        SDL_Log("Failed to write PNG: %s", filename);
                    }

                    SDL_DestroySurface(converted);
                }
                SDL_DestroySurface(frame_surface);
            } else {
                SDL_Log("SDL_RenderReadPixels failed: %s",
                    SDL_GetError());
            }
        }

        // ---------- 显示：离屏纹理叠加到背景上 ----------
        SDL_SetRenderTarget(renderer, nullptr);

        // 窗口清黑（letterbox 黑边）
        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
        SDL_RenderClear(renderer);

        // 在 letterbox 区域内铺上背景色
        int win_w, win_h;
        SDL_GetRenderOutputSize(renderer, &win_w, &win_h);
        float scale = SDL_min(
            (float)win_w / (float)LOGICAL_WIDTH,
            (float)win_h / (float)LOGICAL_HEIGHT);
        float dst_w = (float)LOGICAL_WIDTH * scale;
        float dst_h = (float)LOGICAL_HEIGHT * scale;
        SDL_FRect letterbox_rect = {
            ((float)win_w - dst_w) / 2.0f,
            ((float)win_h - dst_h) / 2.0f,
            dst_w, dst_h
        };

        // 先绘制背景色到 letterbox 区域
        SDL_SetRenderDrawColor(renderer, 16, 16, 32, 255);
        SDL_RenderFillRect(renderer, &letterbox_rect);

        // 离屏纹理使用 BLENDMODE_BLEND_PREMULTIPLIED 叠加到背景上
        // dstRGBA = srcRGBA + dstRGBA * (1 - srcA)
        // 预乘alpha纹理直接相加，alpha自动控制透明度
        SDL_RenderTexture(renderer, offscreen, nullptr, &letterbox_rect);

        SDL_RenderPresent(renderer);
    }

    // ---------- 7. 清理 ----------
    if (mouse_grabbed) {
        SDL_SetWindowRelativeMouseMode(window, false);
    }
    // 清理关键点纹理
    for (auto* tex : index_textures) {
        if (tex) SDL_DestroyTexture(tex);
    }
    for (auto* tex : glo_textures) {
        if (tex) SDL_DestroyTexture(tex);
    }
    for (auto* tex : cam_textures) {
        if (tex) SDL_DestroyTexture(tex);
    }
    for (auto* tex : pix_textures) {
        if (tex) SDL_DestroyTexture(tex);
    }
    if (target_tex_info.texture) SDL_DestroyTexture(target_tex_info.texture);
    SDL_DestroyTexture(offscreen);
    SDL_DestroyTexture(tex_info.texture);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}
