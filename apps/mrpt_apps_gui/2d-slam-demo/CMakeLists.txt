#-----------------------------------------------------------------------------------------------
# CMake file for MRPT app
#-----------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)

project(2d-slam-demo)

find_package(mrpt_common REQUIRED)

find_package(mrpt_slam REQUIRED)
find_package(mrpt_imgui QUIET)
if(NOT mrpt_imgui_FOUND)
  message(STATUS "mrpt_imgui not found (built without GLFW?): skipping ${PROJECT_NAME}")
  return()
endif()
find_package(CLI11 REQUIRED)

mrpt_add_executable(
  TARGET ${PROJECT_NAME}
  SOURCES
    2d-slam-demo_main.cpp
    SlamDemoApp.cpp
    SlamDemoApp.h
    SlamDemoSimulator.cpp
    SlamDemoSimulator.h
  LINK_LIBRARIES
    mrpt::mrpt_imgui
    mrpt::mrpt_slam
    CLI11::CLI11
)
