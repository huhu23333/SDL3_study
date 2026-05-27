#ifndef RENDER_UTILS_H
#define RENDER_UTILS_H

#include <scene_node.h>
#include <camera_projection.h>

#include <SDL3/SDL.h>

#include <string>
#include <vector>

// -----------------------------------------------------------------------------
// 文字渲染（基于 OpenCV）
// -----------------------------------------------------------------------------

/**
 * @brief 使用 OpenCV 渲染文字到带透明通道的 SDL_Texture
 *
 * @param renderer    SDL 渲染器
 * @param text        要渲染的文字
 * @param color       文字颜色
 * @param font_scale  字体缩放（传递给 cv::putText）
 * @param thickness   文字粗细（传递给 cv::putText）
 * @return SDL_Texture*  包含文字且背景透明的纹理，失败返回 nullptr
 */
SDL_Texture* RenderTextToTexture(SDL_Renderer* renderer, const char* text,
                                  SDL_FColor color = { 1.0f, 1.0f, 1.0f, 1.0f },
                                  double font_scale = 0.5,
                                  int thickness = 1);

// -----------------------------------------------------------------------------
// 场景节点快速创建
// -----------------------------------------------------------------------------

/**
 * @brief 创建并初始化一个 ImageNode，添加到场景中
 *
 * @param scene          场景对象
 * @param texture        纹理指针
 * @param tex_w, tex_h   纹理像素尺寸
 * @param display_w, display_h  世界坐标显示尺寸
 * @param pos_x, pos_y, pos_z   局部坐标位置
 * @param alpha          透明度 [0, 1]
 * @param border_width   边框宽度（0 无边框）
 * @param border_color   边框颜色
 * @param keypoints      关键点列表（可空）
 * @param parent         父节点指针（nullptr 表示无父节点）
 * @return ImageNode*    创建的节点指针
 */
ImageNode* CreateImageNode(
    Scene& scene,
    SDL_Texture* texture, int tex_w, int tex_h,
    double display_w, double display_h,
    double pos_x, double pos_y, double pos_z,
    float alpha = 1.0f,
    double border_width = 0.0,
    SDL_FColor border_color = { 1.0f, 1.0f, 0.39f, 1.0f },
    const std::vector<Keypoint>& keypoints = {},
    SceneNode* parent = nullptr);

// -----------------------------------------------------------------------------
// 渲染步骤辅助函数
// -----------------------------------------------------------------------------

/**
 * @brief 开始离屏渲染：设置渲染目标为离屏纹理，并清空为透明
 */
void BeginOffscreenRender(SDL_Renderer* renderer, SDL_Texture* offscreen);

/**
 * @brief 在指定位置绘制十字丝
 *
 * @param renderer     SDL 渲染器
 * @param cx, cy       十字丝中心（像素坐标）
 * @param color        颜色
 * @param half_length  半长
 * @param thickness    线宽
 */
void DrawCrosshair(SDL_Renderer* renderer, float cx, float cy,
                   SDL_FColor color = { 0.39f, 1.0f, 0.39f, 1.0f },
                   float half_length = 20.0f, float thickness = 4.0f);

/**
 * @brief 将离屏纹理呈现到窗口（含 letterbox 适配和背景填充）
 *
 * @param renderer       SDL 渲染器
 * @param offscreen      离屏纹理
 * @param logical_w, logical_h  逻辑尺寸（离屏纹理创建时的宽高）
 * @param bg_color       letterbox 背景色
 */
void PresentOffscreenToWindow(SDL_Renderer* renderer, SDL_Texture* offscreen,
                              int logical_w, int logical_h,
                              SDL_FColor bg_color = { 16.0f/255.0f, 16.0f/255.0f, 32.0f/255.0f, 1.0f });

// -----------------------------------------------------------------------------
// 关键点附加纹理管理（外部缓存）
// -----------------------------------------------------------------------------

/**
 * @brief 构建关键点所有附加纹理（含序号 + 坐标标签），已合并可直接传入 RenderKeypoints
 *
 * 遍历所有关键点，计算世界/相机/像素坐标，将序号纹理（静态，来自 index_textures）
 * 与坐标信息文字标签（glo/cam/pix，动态，使用缓存避免重复创建）合并到一个列表中。
 * 输出可直接作为 RenderKeypoints 的 all_extra_textures 参数传入。
 *
 * @param node                    目标 ImageNode（包含关键点）
 * @param renderer                SDL 渲染器
 * @param intrinsics              相机内参
 * @param distortion              畸变系数
 * @param cam_pos                 相机位置
 * @param cam_yaw, cam_pitch, cam_roll  相机姿态
 * @param index_textures          每个关键点的序号纹理（静态，合并时放置到关键点右侧）
 * @param out_all_textures        输出：每个关键点的完整附加纹理列表（序号 + 坐标标签合并）
 * @param cached_glo_texts, cached_glo_textures  全局坐标缓存
 * @param cached_cam_texts, cached_cam_textures  相机坐标缓存
 * @param cached_pix_texts, cached_pix_textures  像素坐标缓存
 */
void BuildKeypointAllTextures(
    const ImageNode& node,
    SDL_Renderer* renderer,
    const CameraIntrinsics& intrinsics,
    const DistortionCoefficients& distortion,
    const Point3D& cam_pos,
    double cam_yaw, double cam_pitch, double cam_roll,
    const std::vector<SDL_Texture*>& index_textures,
    std::vector<std::vector<ExtraTextureInfo>>& out_all_textures,
    std::vector<std::string>& cached_glo_texts,
    std::vector<SDL_Texture*>& cached_glo_textures,
    std::vector<std::string>& cached_cam_texts,
    std::vector<SDL_Texture*>& cached_cam_textures,
    std::vector<std::string>& cached_pix_texts,
    std::vector<SDL_Texture*>& cached_pix_textures);

#endif // RENDER_UTILS_H
