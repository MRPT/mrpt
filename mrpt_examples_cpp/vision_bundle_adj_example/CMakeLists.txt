#-----------------------------------------------------------------------------------------------
# CMake file for the MRPT example: vision_bundle_adj_example
# Generated automatically - Manual changes may be overwritten.
#-----------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)

project(vision_bundle_adj_example)

# Basic MRPT infrastructure
find_package(mrpt_common REQUIRED)

# Dependencies
find_package(mrpt_vision_lgpl REQUIRED)
find_package(mrpt_gui REQUIRED)


# Define the executable target:
mrpt_add_executable(
  TARGET vision_bundle_adj_example
  SOURCES main.cpp
  LINK_LIBRARIES mrpt::mrpt_vision_lgpl mrpt::mrpt_gui 
)
