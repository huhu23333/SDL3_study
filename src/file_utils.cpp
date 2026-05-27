#include <file_utils.h>

#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>

#include <stb_image.h>
#include <stb_image_write.h>

// =============================================================================
// 加载 PNG 纹理
// =============================================================================
TextureInfo LoadTextureFromPNG(SDL_Renderer* renderer, const char* filepath)
{
    TextureInfo info;
    info.filepath = filepath;

    int width, height, channels;
    unsigned char* pixel_data = stbi_load(filepath, &width, &height, &channels, 4);
    if (!pixel_data) {
        SDL_Log("stbi_load failed for '%s': %s", filepath, stbi_failure_reason());
        return info;
    }

    SDL_Log("Loaded '%s': %dx%d, %d channels", filepath, width, height, channels);

    info.texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGBA32,
                                     SDL_TEXTUREACCESS_STATIC, width, height);
    if (!info.texture) {
        SDL_Log("SDL_CreateTexture (RGBA32) failed: %s", SDL_GetError());
        stbi_image_free(pixel_data);
        return info;
    }

    SDL_UpdateTexture(info.texture, nullptr, pixel_data, width * 4);
    stbi_image_free(pixel_data);

    info.width  = width;
    info.height = height;
    return info;
}

// =============================================================================
// 加载关键点
// =============================================================================
std::vector<Keypoint> LoadKeypointsFromFile(const char* filepath)
{
    std::vector<Keypoint> keypoints;
    std::ifstream kf(filepath);
    if (!kf.is_open()) {
        SDL_Log("Warning: Could not open %s", filepath);
        return keypoints;
    }

    std::string line;
    while (std::getline(kf, line)) {
        if (line.empty()) continue;
        std::istringstream iss(line);
        Keypoint kp;
        if (iss >> kp.index >> kp.pixel_x >> kp.pixel_y) {
            keypoints.push_back(kp);
        }
    }
    SDL_Log("Loaded %zu keypoints from %s", keypoints.size(), filepath);
    return keypoints;
}

// =============================================================================
// 截图保存
// =============================================================================
bool SaveScreenshot(SDL_Renderer* renderer, const char* prefix)
{
    SDL_Surface* frame_surface = SDL_RenderReadPixels(renderer, nullptr);
    if (!frame_surface) {
        SDL_Log("SDL_RenderReadPixels failed: %s", SDL_GetError());
        return false;
    }

    SDL_Surface* converted = SDL_ConvertSurface(
        frame_surface, SDL_PIXELFORMAT_RGBA32);
    if (!converted) {
        SDL_Log("SDL_ConvertSurface failed: %s", SDL_GetError());
        SDL_DestroySurface(frame_surface);
        return false;
    }

    int capture_w = converted->w;
    int capture_h = converted->h;
    size_t pitch = static_cast<size_t>(capture_w) * 4;

    std::vector<uint8_t> rgba_pixels(
        static_cast<size_t>(capture_h) * pitch);
    memcpy(rgba_pixels.data(),
           converted->pixels,
           static_cast<size_t>(capture_h) * pitch);

    // 将预乘 alpha 转为直通 alpha
    for (size_t i = 0; i < static_cast<size_t>(capture_w) * capture_h; ++i) {
        uint8_t* p = rgba_pixels.data() + i * 4;
        uint8_t a = p[3];
        if (a > 0 && a < 255) {
            p[0] = (uint8_t)(((uint16_t)p[0] * 255) / a);
            p[1] = (uint8_t)(((uint16_t)p[1] * 255) / a);
            p[2] = (uint8_t)(((uint16_t)p[2] * 255) / a);
        }
    }

    static int screenshot_counter = 0;
    char filename[64];
    std::snprintf(filename, sizeof(filename),
        "%s_%03d.png", prefix, screenshot_counter++);

    bool success = false;
    if (stbi_write_png(filename, capture_w, capture_h,
            4, rgba_pixels.data(),
            static_cast<int>(pitch))) {
        SDL_Log("Screenshot saved: %s (%dx%d)",
            filename, capture_w, capture_h);
        success = true;
    } else {
        SDL_Log("Failed to write PNG: %s", filename);
    }

    SDL_DestroySurface(converted);
    SDL_DestroySurface(frame_surface);
    return success;
}
