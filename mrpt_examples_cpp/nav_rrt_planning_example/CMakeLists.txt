#-----------------------------------------------------------------------------------------------
# CMake file for the MRPT example: nav_rrt_planning_example
# Generated automatically - Manual changes may be overwritten.
#-----------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)

project(nav_rrt_planning_example)

# Basic MRPT infrastructure
find_package(mrpt_common REQUIRED)

# Dependencies
find_package(mrpt_nav REQUIRED)
find_package(mrpt_gui REQUIRED)


# Define the executable target:
mrpt_add_executable(
  TARGET nav_rrt_planning_example
  SOURCES main.cpp
  LINK_LIBRARIES mrpt::mrpt_nav mrpt::mrpt_gui 
)
