#-----------------------------------------------------------------------------------------------
# CMake file for MRPT app
#-----------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)

project(navlog-viewer)

# Basic MRPT infrastructure
find_package(mrpt_common REQUIRED)

# Dependencies
find_package(mrpt_nav REQUIRED)
find_package(mrpt_imgui QUIET)
if(NOT mrpt_imgui_FOUND)
  message(STATUS "mrpt_imgui not found (built without GLFW?): skipping ${PROJECT_NAME}")
  return()
endif()
find_package(CLI11 REQUIRED)

mrpt_add_executable(
  TARGET ${PROJECT_NAME}
  SOURCES
    main.cpp
    navlog-viewer-ui.cpp
    navlog-viewer-ui.h
  LINK_LIBRARIES
    mrpt::mrpt_imgui
    mrpt::mrpt_nav
    CLI11::CLI11
)
