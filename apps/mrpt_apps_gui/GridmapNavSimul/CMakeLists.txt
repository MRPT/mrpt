#-----------------------------------------------------------------------------------------------
# CMake file for MRPT app
#-----------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)

project(GridmapNavSimul)

find_package(mrpt_common REQUIRED)

find_package(mrpt_maps REQUIRED)
find_package(mrpt_kinematics REQUIRED)
find_package(CLI11 REQUIRED)
find_package(mrpt_imgui QUIET)
if(NOT mrpt_imgui_FOUND)
  message(STATUS "mrpt_imgui not found (built without GLFW?): skipping ${PROJECT_NAME}")
  return()
endif()

mrpt_add_executable(
  TARGET ${PROJECT_NAME}
  SOURCES
    GridmapNavSimul_main.cpp
    virtual_map1.xpm
  LINK_LIBRARIES
    mrpt::mrpt_imgui
    mrpt::mrpt_maps
    mrpt::mrpt_kinematics
    CLI11::CLI11
)
