#-----------------------------------------------------------------------------------------------
# CMake file for MRPT app
#-----------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)

project(SceneViewer3D)

# Basic MRPT infrastructure
find_package(mrpt_common REQUIRED)

# Dependencies
find_package(mrpt_opengl REQUIRED)
find_package(mrpt_maps REQUIRED)
find_package(CLI11 REQUIRED)
find_package(mrpt_imgui QUIET)
if(NOT mrpt_imgui_FOUND)
  message(STATUS "mrpt_imgui not found (built without GLFW?): skipping ${PROJECT_NAME}")
  return()
endif()

# Define the executable target:
mrpt_add_executable(
  TARGET ${PROJECT_NAME}
  SOURCES
    SceneViewer3D_main.cpp
    SceneViewerApp.cpp
    SceneViewerApp.h
  LINK_LIBRARIES
    mrpt::mrpt_imgui
    mrpt::mrpt_opengl
    mrpt::mrpt_maps
    CLI11::CLI11
)
