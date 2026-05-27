#ifndef FILE_UTILS_H
#define FILE_UTILS_H

#include <scene_node.h>

#include <SDL3/SDL.h>

#include <string>
#include <vector>

// -----------------------------------------------------------------------------
// 纹理加载信息
// -----------------------------------------------------------------------------
struct TextureInfo {
    SDL_Texture* texture = nullptr;
    int width  = 0;
    int height = 0;
    std::string filepath;
};

// -----------------------------------------------------------------------------
// 文件加载 / 保存函数
// -----------------------------------------------------------------------------

/**
 * @brief 使用 stb_image 加载 PNG 并创建 SDL_Texture
 *
 * @param renderer  SDL 渲染器
 * @param filepath  PNG 文件路径
 * @return TextureInfo  包含 texture 指针、宽、高及文件路径的结构体。
 *                      若加载失败，texture 字段为 nullptr。
 */
TextureInfo LoadTextureFromPNG(SDL_Renderer* renderer, const char* filepath);

/**
 * @brief 从文本文件加载关键点
 *
 * 文件格式：每行 "index pixel_x pixel_y"
 *
 * @param filepath  关键点文件路径
 * @return std::vector<Keypoint>  关键点列表（文件不存在或解析失败时返回空列表）
 */
std::vector<Keypoint> LoadKeypointsFromFile(const char* filepath);

/**
 * @brief 捕获当前渲染内容并保存为 PNG 文件
 *
 * 使用内部静态计数器为文件自动编号（如 screenshot_000.png）。
 *
 * @param renderer  SDL 渲染器（当前需要截图的渲染上下文）
 * @param prefix    文件名前缀（会自动添加序号）
 * @return true     截图保存成功
 * @return false    截图失败
 */
bool SaveScreenshot(SDL_Renderer* renderer, const char* prefix = "screenshot");

#endif // FILE_UTILS_H
