#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>
#include <array>
#include <vector>

#include <performance.h>


void PrintSupportedTextureFormats(SDL_Renderer* renderer)
{
    SDL_PixelFormat* texture_format = static_cast<SDL_PixelFormat*>(SDL_GetPointerProperty(
        SDL_GetRendererProperties(renderer), SDL_PROP_RENDERER_TEXTURE_FORMATS_POINTER, nullptr));
    int index = 0;
    while (texture_format && *texture_format != SDL_PIXELFORMAT_UNKNOWN) {
        SDL_Log("Texture format[%d]: %s", index, SDL_GetPixelFormatName(*texture_format));
        ++texture_format;
        ++index;
    }
}


int main(int argc, char* argv[])
{
    SDL_SetLogPriorities(SDL_LOG_PRIORITY_VERBOSE);

    if (!SDL_Init(SDL_INIT_VIDEO)) {
        SDL_Log("SDL_Init failed: %s", SDL_GetError());
        return -1;
    }

    SDL_Window* window =
        SDL_CreateWindow("Hello, SDL3!", 800, 600, SDL_WINDOW_RESIZABLE);
    if (!window) {
        SDL_Log("Could not create a window: %s", SDL_GetError());
        return -1;
    }

    SDL_SetWindowPosition(window, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED);

    // SDL_Renderer* renderer = SDL_CreateRenderer(window, nullptr);
    // SDL_Renderer* renderer = SDL_CreateRenderer(window, "gpu");
    // SDL_Renderer* renderer = SDL_CreateRenderer(window, "opengl");
    SDL_Renderer* renderer = SDL_CreateRenderer(window, "vulkan");
    if (!renderer) {
        SDL_Log("Create renderer failed: %s", SDL_GetError());
        return -1;
    }

    std::array<SDL_Vertex, 3> origin_vertices = {
        SDL_Vertex { { 150, 100 }, { 1.0f, 0.0f, 0.0f, 1.0f } }, // top
        SDL_Vertex { { 000, 300 }, { 0.0f, 1.0f, 0.0f, 1.0f } }, // left bottom
        SDL_Vertex { { 300, 300 }, { 0.0f, 0.0f, 1.0f, 1.0f } } // right bottom
    };

    // SDL_RenderGeometry(renderer, nullptr, origin_vertices.data(), origin_vertices.size(), nullptr, 0);

    SDL_Event event {};
    bool keep_going = true;

    uint64_t last_tickets = SDL_GetTicks();
    float position = 0.0f;
    float direction = 1.0f;

    int count = SDL_GetNumRenderDrivers();
    for (int i = 0; i < count; ++i) {
        const char* name = SDL_GetRenderDriver(i);
        SDL_Log("Render driver[%d]: %s", i, name);
    }

    auto performance = Performance();

    bool disable_vsync = true;
    int vsync = disable_vsync ? 0 : 1;
    SDL_SetRenderVSync(renderer, vsync);
    SDL_Log("VSync: %d", vsync);

    PrintSupportedTextureFormats(renderer);

    while (keep_going) {
        uint64_t current_ticks = SDL_GetTicks();
        float delta_time = (current_ticks - last_tickets) / 1000.0f;
        last_tickets = current_ticks;
        position += 200.0f * delta_time * direction;

        int width = 0;
        SDL_GetRenderOutputSize(renderer, &width, nullptr);
        float max_position = static_cast<float>(width) - (origin_vertices[2].position.x - origin_vertices[1].position.x);

        if (position > max_position) {
            direction = -1.0f;
        } else if (position < 0.0f) {
            position = 0.0f;
            direction = 1.0f;
        }

        std::vector<SDL_Vertex> vertices;
        for (const auto& vertex : origin_vertices) {
            vertices.push_back(vertex);
            vertices.back().position.x += position;
        }


        while (SDL_PollEvent(&event)) {
            switch (event.type) {
            case SDL_EVENT_QUIT:
                keep_going = false;
                break;

            case SDL_EVENT_KEY_DOWN: {
                keep_going = keep_going && (event.key.key != SDLK_ESCAPE);
                break;
            }
            }
        }

        SDL_SetRenderDrawColor(renderer, 32, 0, 32, 255);
        SDL_RenderClear(renderer);
        //SDL_RenderGeometry(renderer, nullptr, origin_vertices.data(), origin_vertices.size(), nullptr, 0);
        SDL_RenderGeometry(renderer, nullptr, vertices.data(), vertices.size(), nullptr, 0);
        SDL_RenderPresent(renderer);

        performance.IncreaseFrameCount();
        performance.PrintEverySecond();
    }

    // while (keep_going) {
    //     SDL_WaitEvent(&event);

    //     switch (event.type) {
    //         case SDL_EVENT_QUIT: {
    //             keep_going = false;
    //             break;
    //         }

    //         case SDL_EVENT_KEY_DOWN: {
    //             keep_going = keep_going && (event.key.key != SDLK_ESCAPE);
    //             break;
    //         }

    //         case SDL_EVENT_WINDOW_EXPOSED: {
    //             SDL_SetRenderDrawColor(renderer, 16, 0, 16, 255);
    //             SDL_RenderClear(renderer);
    //             SDL_RenderPresent(renderer);
    //             break;
    //         }
    //     }
    //     SDL_Log("Event: %d", event.type);
    // }

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}