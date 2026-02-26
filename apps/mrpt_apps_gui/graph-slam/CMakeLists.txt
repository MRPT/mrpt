#-----------------------------------------------------------------------------------------------
# CMake file for MRPT app
#-----------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)

project(graph-slam)

# Basic MRPT infrastructure
find_package(mrpt_common REQUIRED)

# Dependencies
find_package(mrpt_graphslam REQUIRED)
find_package(mrpt_gui REQUIRED)
find_package(CLI11 REQUIRED)

# Define the executable target:
mrpt_add_executable(
  TARGET ${PROJECT_NAME}
  SOURCES graph-slam_main.cpp display_graph.cpp
  LINK_LIBRARIES 
    mrpt::mrpt_graphslam
    mrpt::mrpt_gui
    CLI11::CLI11
)
