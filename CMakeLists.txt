cmake_minimum_required(VERSION 3.1)
project(example)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# libigl
option(LIBIGL_WITH_OPENGL            "Use OpenGL"         ON)
option(LIBIGL_WITH_OPENGL_GLFW       "Use GLFW"           ON)
#option(LIBIGL_WITH_OPENGL_GLFW_IMGUI "Use ImGui"          ON)

find_package(LIBIGL REQUIRED QUIET)

# Add your project files
add_executable(main_bin main.cpp)
target_link_libraries(main_bin igl::core igl::opengl_glfw)

add_executable(smoothing_bin smoothing.cpp)
target_link_libraries(smoothing_bin igl::core igl::opengl_glfw)
