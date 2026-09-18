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

# 2) Dear ImGui + ImPlot + GLFW, from the single vendored copy in the repo.
#    That package skips itself when glfw3 is absent (e.g. the GUI-disabled
#    Windows CI), so skip this optional example too instead of failing the
#    whole mrpt_examples_cpp build.
find_package(mrpt_imgui_vendor QUIET)
if (NOT mrpt_imgui_vendor_FOUND)
  message(STATUS "imgui_mrpt_example: mrpt_imgui_vendor not found, skipping this example.")
  return()
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
    mrpt::mrpt_imgui_vendor  # glfw + OpenGL::GL come transitively
)
