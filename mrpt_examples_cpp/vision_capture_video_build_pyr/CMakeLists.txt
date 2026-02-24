#-----------------------------------------------------------------------------------------------
# CMake file for the MRPT example: vision_capture_video_build_pyr
# Generated automatically - Manual changes may be overwritten.
#-----------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)

project(vision_capture_video_build_pyr)

# Basic MRPT infrastructure
find_package(mrpt_common REQUIRED)

# Dependencies
find_package(mrpt_vision REQUIRED)
find_package(mrpt_hwdrivers REQUIRED)


# Define the executable target:
mrpt_add_executable(
  TARGET vision_capture_video_build_pyr
  SOURCES main.cpp
  LINK_LIBRARIES mrpt::mrpt_vision mrpt::mrpt_hwdrivers 
)
