#include <render_utils.h>

#include <cstdio>
#include <cmath>

#include <opencv2/opencv.hpp>

// =============================================================================
// 文字渲染（OpenCV → SDL_Texture）
// =============================================================================
SDL_Texture* RenderTextToTexture(SDL_Renderer* renderer, const char* text,
                                  SDL_FColor color,
                                  double font_scale,
                                  int thickness)
{
    (void)color;  // 颜色在 opencv 渲染时用纯白，通过 SDL 纹理 blend 模式调色

    int baseline = 0;
    cv::Size text_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX,
                                          font_scale, thickness, &baseline);
    if (text_size.width <= 0 || text_size.height <= 0) return nullptr;

    int pad = 2;
    int w = text_size.width + pad * 2;
    int h = text_size.height + baseline + pad * 2;

    cv::Mat img(h, w, CV_8UC4, cv::Scalar(0, 0, 0, 0));

    cv::putText(img, text, cv::Point(pad, h - pad - baseline),
                cv::FONT_HERSHEY_SIMPLEX, font_scale,
                cv::Scalar(255, 255, 255, 255), thickness, cv::LINE_AA);

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

// =============================================================================
// 场景节点快速创建
// =============================================================================
ImageNode* CreateImageNode(
    Scene& scene,
    SDL_Texture* texture, int tex_w, int tex_h,
    double display_w, double display_h,
    double pos_x, double pos_y, double pos_z,
    float alpha,
    double border_width,
    SDL_FColor border_color,
    const std::vector<Keypoint>& keypoints,
    SceneNode* parent)
{
    auto img_node = std::make_shared<ImageNode>();

    img_node->SetTexture(texture, tex_w, tex_h);
    img_node->SetDisplaySize(display_w, display_h);
    img_node->SetLocalPosition(pos_x, pos_y, pos_z);
    img_node->SetAlpha(alpha);
    img_node->SetBorderWidth(border_width);
    img_node->SetBorderColor(border_color);

    if (!keypoints.empty()) {
        img_node->SetKeypoints(keypoints);
    }

    SceneNode* added_ptr = scene.AddNode(std::move(img_node));

    if (parent) {
        added_ptr->SetParent(parent);
    }

    return dynamic_cast<ImageNode*>(added_ptr);
}

// =============================================================================
// 渲染步骤辅助函数
// =============================================================================
void BeginOffscreenRender(SDL_Renderer* renderer, SDL_Texture* offscreen)
{
    SDL_SetRenderTarget(renderer, offscreen);
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 0);
    SDL_RenderClear(renderer);
}

void DrawCrosshair(SDL_Renderer* renderer, float cx, float cy,
                   SDL_FColor color, float half_length, float thickness)
{
    // 水平条
    SDL_Vertex h_verts[6] = {
        { { cx - half_length, cy - thickness }, color, { 0, 0 } },
        { { cx + half_length, cy - thickness }, color, { 0, 0 } },
        { { cx - half_length, cy + thickness }, color, { 0, 0 } },
        { { cx + half_length, cy - thickness }, color, { 0, 0 } },
        { { cx + half_length, cy + thickness }, color, { 0, 0 } },
        { { cx - half_length, cy + thickness }, color, { 0, 0 } }
    };
    SDL_RenderGeometry(renderer, nullptr, h_verts, 6, nullptr, 0);

    // 垂直条
    SDL_Vertex v_verts[6] = {
        { { cx - thickness, cy - half_length }, color, { 0, 0 } },
        { { cx + thickness, cy - half_length }, color, { 0, 0 } },
        { { cx - thickness, cy + half_length }, color, { 0, 0 } },
        { { cx + thickness, cy - half_length }, color, { 0, 0 } },
        { { cx + thickness, cy + half_length }, color, { 0, 0 } },
        { { cx - thickness, cy + half_length }, color, { 0, 0 } }
    };
    SDL_RenderGeometry(renderer, nullptr, v_verts, 6, nullptr, 0);
}

void PresentOffscreenToWindow(SDL_Renderer* renderer, SDL_Texture* offscreen,
                              int logical_w, int logical_h,
                              SDL_FColor bg_color)
{
    SDL_SetRenderTarget(renderer, nullptr);

    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    SDL_RenderClear(renderer);

    int win_w, win_h;
    SDL_GetRenderOutputSize(renderer, &win_w, &win_h);
    float scale = SDL_min(
        (float)win_w / (float)logical_w,
        (float)win_h / (float)logical_h);
    float dst_w = (float)logical_w * scale;
    float dst_h = (float)logical_h * scale;
    SDL_FRect letterbox_rect = {
        ((float)win_w - dst_w) / 2.0f,
        ((float)win_h - dst_h) / 2.0f,
        dst_w, dst_h
    };

    SDL_SetRenderDrawColor(renderer,
        (uint8_t)(bg_color.r * 255),
        (uint8_t)(bg_color.g * 255),
        (uint8_t)(bg_color.b * 255),
        (uint8_t)(bg_color.a * 255));
    SDL_RenderFillRect(renderer, &letterbox_rect);

    SDL_RenderTexture(renderer, offscreen, nullptr, &letterbox_rect);

    SDL_RenderPresent(renderer);
}

// =============================================================================
// 关键点所有附加纹理构建（含序号 + 坐标标签，已合并）
// =============================================================================
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
    std::vector<SDL_Texture*>& cached_pix_textures)
{
    const auto& keypoints = node.GetKeypoints();
    const float MAX_COORD = 1e6f;

    out_all_textures.resize(keypoints.size());

    for (size_t ki = 0; ki < keypoints.size(); ++ki) {
        out_all_textures[ki].clear();

        Point3D world_pt = node.GetKeypointWorldPos(ki);
        Point3D cam_pt = WorldToCameraTransform(world_pt, cam_pos,
                                                  cam_yaw, cam_pitch, cam_roll);
        if (cam_pt.z <= 0.001) continue;

        Point2D screen_pt = ProjectPoint(cam_pt, intrinsics, distortion);
        if (std::isnan(screen_pt.x) || std::isnan(screen_pt.y) ||
            std::abs(screen_pt.x) > MAX_COORD || std::abs(screen_pt.y) > MAX_COORD)
            continue;

        float sx = (float)screen_pt.x;
        float sy = (float)screen_pt.y;

        // --- 1. 序号纹理（静态，来自外部） ---
        if (ki < index_textures.size() && index_textures[ki]) {
            float tw, th;
            SDL_GetTextureSize(index_textures[ki], &tw, &th);
            out_all_textures[ki].push_back(
                { index_textures[ki], sx + 10.0f, sy - th / 2.0f });
        }

        // --- 2. 坐标信息文字标签 ---
        char glo_str[64], cam_str[64], pix_str[64];
        std::snprintf(glo_str, sizeof(glo_str), "glo:(%.2f, %.2f, %.2f)",
                     world_pt.x, world_pt.y, world_pt.z);
        std::snprintf(cam_str, sizeof(cam_str), "cam:(%.2f, %.2f, %.2f)",
                     cam_pt.x, cam_pt.y, cam_pt.z);
        std::snprintf(pix_str, sizeof(pix_str), "pix:(%.1f, %.1f)",
                     screen_pt.x, screen_pt.y);

        auto update_texture = [&](const char* new_text,
                                  std::string& cached_text,
                                  SDL_Texture*& cached_texture,
                                  SDL_FColor text_color) -> SDL_Texture* {
            if (cached_text != new_text) {
                cached_text = new_text;
                if (cached_texture) {
                    SDL_DestroyTexture(cached_texture);
                    cached_texture = nullptr;
                }
                cached_texture = RenderTextToTexture(renderer, new_text,
                                                      text_color, 0.55, 2);
            }
            return cached_texture;
        };

        float line_y = sy + 10.0f;

        // --- glo ---
        if (ki < cached_glo_texts.size()) {
            SDL_Texture* tex = update_texture(glo_str,
                cached_glo_texts[ki], cached_glo_textures[ki],
                { 0.4f, 1.0f, 0.4f, 1.0f });
            if (tex) {
                float tw, th;
                SDL_GetTextureSize(tex, &tw, &th);
                out_all_textures[ki].push_back({ tex, sx + 14.0f, line_y });
                line_y += th + 2.0f;
            }
        }

        // --- cam ---
        if (ki < cached_cam_texts.size()) {
            SDL_Texture* tex = update_texture(cam_str,
                cached_cam_texts[ki], cached_cam_textures[ki],
                { 0.4f, 0.8f, 1.0f, 1.0f });
            if (tex) {
                float tw, th;
                SDL_GetTextureSize(tex, &tw, &th);
                out_all_textures[ki].push_back({ tex, sx + 14.0f, line_y });
                line_y += th + 2.0f;
            }
        }

        // --- pix ---
        if (ki < cached_pix_texts.size()) {
            SDL_Texture* tex = update_texture(pix_str,
                cached_pix_texts[ki], cached_pix_textures[ki],
                { 1.0f, 1.0f, 0.4f, 1.0f });
            if (tex) {
                float tw, th;
                SDL_GetTextureSize(tex, &tw, &th);
                out_all_textures[ki].push_back({ tex, sx + 14.0f, line_y });
            }
        }
    }
}
