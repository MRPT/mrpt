cmake_minimum_required(VERSION 3.16)
project(imgui_mrpt_example LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---------------------------------------------------------------
# Dependencies
# ---------------------------------------------------------------

# 1) MRPT (>=3.0)
find_package(mrpt-gui  REQUIRED)  # brings in mrpt-viz transitively
find_package(mrpt-opengl REQUIRED)

# 2) Dear ImGui; fetched via FetchContent for reproducibility.
#    If you have a system / vcpkg install, replace this block.
include(FetchContent)

FetchContent_Declare(
    imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG        docking  # adjust to latest stable
)
FetchContent_MakeAvailable(imgui)

# 3) GLFW
find_package(glfw3 3.3 REQUIRED)

# 4) OpenGL
find_package(OpenGL REQUIRED)

# ---------------------------------------------------------------
# Build the Dear ImGui library (core + glfw + opengl3 backends)
# ---------------------------------------------------------------
set(IMGUI_SRC_DIR "${imgui_SOURCE_DIR}")

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

target_compile_definitions(imgui_lib PUBLIC
  IMGUI_IMPL_OPENGL_ES3
)

# ImGui needs to know which loader to use — we rely on the system GL
# headers directly (GL 3.3 core is enough).
target_compile_definitions(imgui_lib PUBLIC
    IMGUI_IMPL_OPENGL_LOADER_CUSTOM   # we don't use glad/glew from imgui
)

# ---------------------------------------------------------------
# Demo executable
# ---------------------------------------------------------------
add_executable(${PROJECT_NAME}
    ${PROJECT_NAME}.cpp
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    mrpt::opengl
    mrpt::gui
    #glfw        # transitive from imgui_lib
    #OpenGL::GL  # transitive from imgui_lib
    imgui_lib
)
