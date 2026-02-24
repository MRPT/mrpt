#-----------------------------------------------------------------------------------------------
# CMake file for the MRPT example: hwdrivers_taobotics_imu
# Generated automatically - Manual changes may be overwritten.
#-----------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)

project(hwdrivers_taobotics_imu)

# Basic MRPT infrastructure
find_package(mrpt_common REQUIRED)

# Dependencies
find_package(mrpt_hwdrivers REQUIRED)
find_package(mrpt_maps REQUIRED)


# Define the executable target:
mrpt_add_executable(
  TARGET hwdrivers_taobotics_imu
  SOURCES main.cpp
  LINK_LIBRARIES mrpt::mrpt_hwdrivers mrpt::mrpt_maps 
)
