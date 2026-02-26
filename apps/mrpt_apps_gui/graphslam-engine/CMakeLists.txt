

#-----------------------------------------------------------------------------------------------
# CMake file for MRPT app
#-----------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)

project(graphslam-engine)

# Basic MRPT infrastructure
find_package(mrpt_common REQUIRED)

# Dependencies
find_package(mrpt_graphslam REQUIRED)
find_package(mrpt_graphs REQUIRED)
find_package(mrpt_gui REQUIRED)
find_package(mrpt_obs REQUIRED)
find_package(mrpt_maps REQUIRED)
find_package(mrpt_slam REQUIRED)
find_package(mrpt_opengl REQUIRED)
find_package(CLI11 REQUIRED)

# Define the executable target:
mrpt_add_executable(
  TARGET ${PROJECT_NAME}
  SOURCES graphslam-engine_app.cpp
  LINK_LIBRARIES 
    mrpt::mrpt_graphslam
    mrpt::mrpt_graphs
    mrpt::mrpt_gui
    mrpt::mrpt_obs
    mrpt::mrpt_maps
    mrpt::mrpt_slam
    mrpt::mrpt_opengl
    CLI11::CLI11
)
