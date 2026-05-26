cmake_minimum_required(VERSION 3.16)

project(SDL3_study)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_BUILD_TYPE Release)

# set the output directory for built objects.
# This makes sure that the dynamic library goes into the build directory automatically.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/$<CONFIGURATION>")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/$<CONFIGURATION>")

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/third_party
)

add_library(my_study_lib SHARED
    src/performance.cpp
    src/camera_projection.cpp
    src/stb_image_impl.cpp
)

# This assumes the SDL source is available in vendored/SDL
add_subdirectory(third_party/SDL3 EXCLUDE_FROM_ALL)

# Create your game executable target as usual
add_executable(my_study_app src/main.cpp)

# Camera test executable (console only, no SDL needed)
add_executable(camera_test src/main_camera_test.cpp)

# Link to the actual SDL3 library.
target_link_libraries(my_study_app 
    SDL3::SDL3
    my_study_lib
)

target_link_libraries(camera_test
    my_study_lib
)

# 3D textured quad demo executable
add_executable(textured_quad src/main_textured_quad.cpp)

find_package(OpenCV REQUIRED)

target_link_libraries(textured_quad
    SDL3::SDL3
    my_study_lib
    ${OpenCV_LIBS}
)

