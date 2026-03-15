#-----------------------------------------------------------------------------------------------
# CMake file for the MRPT example: hwdrivers_camera_capture_dialog
# Generated automatically - Manual changes may be overwritten.
#-----------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)

project(hwdrivers_camera_capture_dialog)

# Basic MRPT infrastructure
find_package(mrpt_common REQUIRED)

# Dependencies
find_package(mrpt_hwdrivers REQUIRED)
find_package(mrpt_libapps_gui REQUIRED)


# Define the executable target:
mrpt_add_executable(
  TARGET hwdrivers_camera_capture_dialog
  SOURCES main.cpp
  LINK_LIBRARIES mrpt::mrpt_hwdrivers mrpt::mrpt_libapps_gui 
)
