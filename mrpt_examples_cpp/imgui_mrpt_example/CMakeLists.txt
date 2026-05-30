cmake_minimum_required(VERSION 3.16)
project(imgui_mrpt_example LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---------------------------------------------------------------
# Dependencies
# ---------------------------------------------------------------

# 1) MRPT (>=3.0)
find_package(mrpt_imgui  REQUIRED)  # brings in mrpt-viz transitively
find_package(mrpt_opengl REQUIRED)

# 2) Dear ImGui — vendored copy (docking branch, commit e05bd9deb, v1.92.7 WIP)
#    Source files live under imgui/ within this example directory.
#    License: MIT (see imgui/LICENSE.txt)

# 3) GLFW
find_package(glfw3 3.3 REQUIRED)

# 4) OpenGL
find_package(OpenGL REQUIRED)

# ---------------------------------------------------------------
# Build the Dear ImGui library (core + glfw + opengl3 backends)
# ---------------------------------------------------------------
set(IMGUI_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/imgui")

add_library(imgui_lib STATIC
    ${IMGUI_SRC_DIR}/imgui.cpp
    ${IMGUI_SRC_DIR}/imgui_draw.cpp
    ${IMGUI_SRC_DIR}/imgui_tables.cpp
    ${IMGUI_SRC_DIR}/imgui_widgets.cpp
    ${IMGUI_SRC_DIR}/imgui_demo.cpp
    ${IMGUI_SRC_DIR}/backends/imgui_impl_glfw.cpp
    ${IMGUI_SRC_DIR}/backends/imgui_impl_opengl3.cpp
)

target_include_directories(imgui_lib PUBLIC
    ${IMGUI_SRC_DIR}
    ${IMGUI_SRC_DIR}/backends
)

target_link_libraries(imgui_lib PUBLIC
    glfw
    OpenGL::GL
)

# GLES3 is only available on Linux/Android/embedded; macOS uses standard OpenGL
if (NOT APPLE)
  target_compile_definitions(imgui_lib PUBLIC IMGUI_IMPL_OPENGL_ES3)
  # On Linux with custom loader, rely on system GL headers directly
  target_compile_definitions(imgui_lib PUBLIC IMGUI_IMPL_OPENGL_LOADER_CUSTOM)
else()
  # On macOS, use imgui's own embedded loader which handles Apple OpenGL setup
  target_include_directories(imgui_lib PUBLIC ${OPENGL_INCLUDE_DIR})
endif()

# ---------------------------------------------------------------
# Demo executable
# ---------------------------------------------------------------
mrpt_add_executable(
  TARGET ${PROJECT_NAME}
  SOURCES ${PROJECT_NAME}.cpp
  LINK_LIBRARIES
    mrpt::mrpt_opengl
    mrpt::mrpt_imgui
    #glfw        # transitive from imgui_lib
    #OpenGL::GL  # transitive from imgui_lib
    imgui_lib
)
