cmake_minimum_required(VERSION 3.16)

project(demo)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_BUILD_TYPE Release)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/$<CONFIGURATION>")

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/third_party
)

# Build SDL3 from source (the only external dependency)
add_subdirectory(third_party/SDL3)

find_package(OpenCV REQUIRED)

# Main executable — all source files compiled in directly
add_executable(demo
    src/demo.cpp
    src/camera_projection.cpp
    src/stb_image_impl.cpp
    src/scene_node.cpp
    src/file_utils.cpp
    src/render_utils.cpp
)

target_link_libraries(demo
    SDL3::SDL3
    ${OpenCV_LIBS}
)
