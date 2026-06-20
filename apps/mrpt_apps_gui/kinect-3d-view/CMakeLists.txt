

#-----------------------------------------------------------------------------------------------
# CMake file for MRPT app
#-----------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)

project(kinect-3d-view)

# Basic MRPT infrastructure
find_package(mrpt_common REQUIRED)

# Dependencies
find_package(mrpt_gui REQUIRED)
find_package(mrpt_hwdrivers REQUIRED)
find_package(mrpt_maps REQUIRED)

# Define the executable target:
mrpt_add_executable(
  TARGET ${PROJECT_NAME}
  SOURCES kinect-3d-view_main.cpp
  LINK_LIBRARIES
    mrpt::mrpt_gui
    mrpt::mrpt_hwdrivers
    mrpt::mrpt_maps
)
