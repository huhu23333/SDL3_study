#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>
#include <camera_projection.h>
#include <scene_node.h>
#include <file_utils.h>
#include <render_utils.h>

#include <cstdio>
#include <cmath>
#include <cstdlib>
#include <vector>
#include <algorithm>

#include <cstring>
#include <string>

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
        "Demo - Scene Node System",
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

    SDL_Texture* offscreen = SDL_CreateTexture(
        renderer, SDL_PIXELFORMAT_RGBA32,
        SDL_TEXTUREACCESS_TARGET, LOGICAL_WIDTH, LOGICAL_HEIGHT);
    if (!offscreen) {
        SDL_Log("Create offscreen texture failed: %s", SDL_GetError());
    }
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
    std::vector<Keypoint> keypoints = LoadKeypointsFromFile("images/target.txt");

    // ---------- 3. 设置相机参数 ----------
    CameraIntrinsics intrinsics{
        960, 960,
        LOGICAL_WIDTH / 2.0, LOGICAL_HEIGHT / 2.0,
        LOGICAL_WIDTH, LOGICAL_HEIGHT
    };
    DistortionCoefficients distortion{0.0, 0.0, 0.0, 0.0, 0.0};

    double max_half_fov = ComputeMaxHalfFovAngle(intrinsics, distortion);
    SDL_Log("Max half FOV angle (w/ distortion): %.2f deg",
            max_half_fov * 180.0 / M_PI);
    double diagonal_fov = 2.0 * std::atan(
        std::sqrt((double)(LOGICAL_WIDTH * LOGICAL_WIDTH +
                           LOGICAL_HEIGHT * LOGICAL_HEIGHT)) * 0.5 / 960.0);
    SDL_Log("Diagonal FOV: %.2f deg",
            diagonal_fov * 180.0 / M_PI);

    // ---------- 4. 构建场景节点系统 ----------
    Scene scene;
    double base_height = 2.0;
    double base_width = base_height * (double)tex_info.width / (double)tex_info.height;
    double rect_depth = 2.0;

    // ===== 4a. 前方纹理图像 =====
    CreateImageNode(scene,
                    tex_info.texture, tex_info.width, tex_info.height,
                    base_width, base_height,
                    0.0, 0.0, rect_depth,
                    1.0f, 0.04f, { 1.0f, 1.0f, 0.39f, 1.0f },
                    {}, nullptr);

    // ===== 4b. 后方纹理图像（半透明，可本体旋转） =====
    double back_z = rect_depth + 3.0;
    ImageNode* back_node = CreateImageNode(scene,
                                           tex_info.texture, tex_info.width, tex_info.height,
                                           base_width, base_height,
                                           0.0, 0.0, back_z,
                                           0.7f, 0.0, { 1.0f, 1.0f, 0.39f, 1.0f },
                                           {}, nullptr);

    // ===== 4c. target 图像（作为 BackImage 的子节点） =====
    double target_base_height = 5.0;
    double target_base_width = target_base_height * (double)target_tex_info.width / (double)target_tex_info.height;
    double target_local_z = 6.0;

    ImageNode* target_node = CreateImageNode(scene,
                                             target_tex_info.texture, target_tex_info.width, target_tex_info.height,
                                             target_base_width, target_base_height,
                                             0.0, 0.0, target_local_z,
                                             1.0f, 0.0, { 1.0f, 1.0f, 0.39f, 1.0f },
                                             keypoints, back_node);

    // 预渲染关键点序号纹理
    std::vector<SDL_Texture*> index_textures(keypoints.size(), nullptr);
    for (size_t i = 0; i < keypoints.size(); ++i) {
        char idx_str[16];
        std::snprintf(idx_str, sizeof(idx_str), "%d", keypoints[i].index);
        index_textures[i] = RenderTextToTexture(renderer, idx_str,
                                                  { 0.0f, 1.0f, 0.0f, 1.0f },
                                                  0.8, 2);
    }

    bool show_keypoints = false;

    // ---------- 5. 控制状态 ----------
    bool mouse_grabbed = false;
    struct CameraPose {
        double yaw{0.0};
        double pitch{0.0};
        double roll{0.0};
    };
    CameraPose camera{};
    Point3D cam_pos{ 0.0, 0.0, 0.0 };
    float mouse_sensitivity = 0.002f;
    float move_speed = 2.0f;

    // TargetImage 本体旋转
    double target_yaw = 0.0;
    double target_pitch = 0.0;
    double target_roll = 0.0;

    // BackImage 本体旋转
    double back_yaw = 0.0;
    double back_pitch = 0.0;
    double back_roll = 0.0;

    float obj_rot_speed = 2.0f;

    // true = 旋转 TargetImage, false = 旋转 BackImage
    bool rotate_target_mode = true;

    bool key_w = false, key_s = false, key_a = false, key_d = false;
    bool key_up = false, key_down = false;
    bool key_q = false, key_e = false;
    bool key_j = false, key_l = false;
    bool key_i = false, key_k = false;
    bool key_u = false, key_o = false;

    float roll_speed = 1.5f;
    bool screenshot_requested = false;

    // 关键点附加纹理缓存（由外部管理）
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
    SDL_Log("WASD: move | SPACE: up | SHIFT: down | ESC: release/quit");
    SDL_Log("Q/E: roll camera | R: reset roll");
    SDL_Log("I/J/K/L/U/O: rotate (Yaw/Pitch/Roll)");
    SDL_Log("N: toggle rotation target (TargetImage <-> BackImage)");
    SDL_Log("M: toggle keypoints | P: screenshot");

    uint64_t prev_ticks = SDL_GetTicks();

    while (keep_going) {
        ++frame_count;

        uint64_t current_ticks = SDL_GetTicks();
        float dt = (current_ticks - prev_ticks) / 1000.0f;
        prev_ticks = current_ticks;
        if (dt > 0.05f) dt = 0.05f;

        if (current_ticks - fps_last_ticks >= 1000) {
            SDL_Log("FPS: %d", frame_count);
            frame_count = 0;
            fps_last_ticks = current_ticks;
        }

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
                case SDLK_N:
                    if (event.key.repeat == 0) {
                        rotate_target_mode = !rotate_target_mode;
                        SDL_Log("Rotating: %s",
                                rotate_target_mode ? "TargetImage" : "BackImage");
                    }
                    break;
                case SDLK_M:
                    if (event.key.repeat == 0) {
                        show_keypoints = !show_keypoints;
                        SDL_Log("Keypoints: %s", show_keypoints ? "ON" : "OFF");
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
                    SDL_Log("Mouse captured.");
                }
                break;

            case SDL_EVENT_MOUSE_MOTION:
                if (mouse_grabbed) {
                    camera.yaw   += event.motion.xrel * mouse_sensitivity;
                    camera.pitch -= event.motion.yrel * mouse_sensitivity;
                    const double pitch_limit = 1.5;
                    if (camera.pitch >  pitch_limit) camera.pitch =  pitch_limit;
                    if (camera.pitch < -pitch_limit) camera.pitch = -pitch_limit;
                }
                break;

            case SDL_EVENT_WINDOW_RESIZED:
                break;
            }
        }

        // ---------- 摄像机移动 ----------
        double wf_x = std::sin(camera.yaw);
        double wf_z = std::cos(camera.yaw);
        double wr_x = std::cos(camera.yaw);
        double wr_z = -std::sin(camera.yaw);

        if (key_w) { cam_pos.x += wf_x * move_speed * dt; cam_pos.z += wf_z * move_speed * dt; }
        if (key_s) { cam_pos.x -= wf_x * move_speed * dt; cam_pos.z -= wf_z * move_speed * dt; }
        if (key_a) { cam_pos.x -= wr_x * move_speed * dt; cam_pos.z -= wr_z * move_speed * dt; }
        if (key_d) { cam_pos.x += wr_x * move_speed * dt; cam_pos.z += wr_z * move_speed * dt; }
        if (key_up)   cam_pos.y -= move_speed * dt;
        if (key_down) cam_pos.y += move_speed * dt;

        if (key_q) camera.roll += roll_speed * dt;
        if (key_e) camera.roll -= roll_speed * dt;

        // ---------- 图像旋转（N 键切换目标） ----------
        if (key_j) (rotate_target_mode ? target_yaw : back_yaw)   += obj_rot_speed * dt;
        if (key_l) (rotate_target_mode ? target_yaw : back_yaw)   -= obj_rot_speed * dt;
        if (key_i) (rotate_target_mode ? target_pitch : back_pitch) += obj_rot_speed * dt;
        if (key_k) (rotate_target_mode ? target_pitch : back_pitch) -= obj_rot_speed * dt;
        if (key_u) (rotate_target_mode ? target_roll : back_roll)  += obj_rot_speed * dt;
        if (key_o) (rotate_target_mode ? target_roll : back_roll)  -= obj_rot_speed * dt;

        // ---------- 更新节点变换 ----------
        if (target_node) {
            target_node->SetLocalRotation(target_yaw, target_pitch, target_roll);
        }
        if (back_node) {
            back_node->SetLocalRotation(back_yaw, back_pitch, back_roll);
            back_node->SetLocalPosition(back_yaw, back_pitch, back_roll + back_z);
        }

        // =============================================================
        // 渲染步骤：离屏渲染 → 覆盖层 → 截图 → 显示
        // =============================================================

        // ---- Step 1: 开始离屏渲染 ----
        BeginOffscreenRender(renderer, offscreen);

        // ---- Step 2: 渲染场景节点 ----
        scene.RenderAll(renderer, intrinsics, distortion, cam_pos,
                        camera.yaw, camera.pitch, camera.roll);

        SDL_FlushRenderer(renderer);

        // ---- Step 3: 绘制十字丝 ----
        DrawCrosshair(renderer, (float)intrinsics.cx, (float)intrinsics.cy);

        // ---- Step 4: 关键点渲染 ----
        if (show_keypoints && !keypoints.empty() && target_node) {
            // 在外部完成所有附加纹理的合并（序号 + 坐标标签）
            std::vector<std::vector<ExtraTextureInfo>> all_textures;
            BuildKeypointAllTextures(
                *target_node, renderer,
                intrinsics, distortion,
                cam_pos, camera.yaw, camera.pitch, camera.roll,
                index_textures,
                all_textures,
                cached_glo_texts, glo_textures,
                cached_cam_texts, cam_textures,
                cached_pix_texts, pix_textures);

            // 关键点渲染只传入一个合并后的附加纹理
            target_node->RenderKeypoints(
                renderer, intrinsics, distortion,
                cam_pos, camera.yaw, camera.pitch, camera.roll,
                all_textures);
        }

        // ---- Step 5: 截图 ----
        if (screenshot_requested) {
            screenshot_requested = false;
            SaveScreenshot(renderer, "screenshot");
        }

        // ---- Step 6: 呈现到窗口 ----
        PresentOffscreenToWindow(renderer, offscreen,
                                 LOGICAL_WIDTH, LOGICAL_HEIGHT);
    }

    // ---------- 7. 清理 ----------
    if (mouse_grabbed) {
        SDL_SetWindowRelativeMouseMode(window, false);
    }
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
