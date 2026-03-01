#-----------------------------------------------------------------------------------------------
# CMake file for the MRPT example: hwdrivers_openni2_2d_icp_slam
# Generated automatically - Manual changes may be overwritten.
#-----------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)

project(hwdrivers_openni2_2d_icp_slam)

# Basic MRPT infrastructure
find_package(mrpt_common REQUIRED)

# Dependencies
find_package(mrpt_hwdrivers REQUIRED)
find_package(mrpt_gui REQUIRED)
find_package(mrpt_viz REQUIRED)
find_package(mrpt_maps REQUIRED)


# Define the executable target:
mrpt_add_executable(
  TARGET hwdrivers_openni2_2d_icp_slam
  SOURCES main.cpp
  LINK_LIBRARIES mrpt::mrpt_hwdrivers mrpt::mrpt_gui mrpt::mrpt_viz mrpt::mrpt_maps 
)
