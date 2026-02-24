#-----------------------------------------------------------------------------------------------
# CMake file for the MRPT example: vision_checkerboard_detectors
# Generated automatically - Manual changes may be overwritten.
#-----------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)

project(vision_checkerboard_detectors)

# Basic MRPT infrastructure
find_package(mrpt_common REQUIRED)

# Dependencies
find_package(mrpt_vision REQUIRED)
find_package(mrpt_gui REQUIRED)


# Define the executable target:
mrpt_add_executable(
  TARGET vision_checkerboard_detectors
  SOURCES main.cpp
  LINK_LIBRARIES mrpt::mrpt_vision mrpt::mrpt_gui 
)
