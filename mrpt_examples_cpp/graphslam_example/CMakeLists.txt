#-----------------------------------------------------------------------------------------------
# CMake file for the MRPT example: graphslam_example
# Generated automatically - Manual changes may be overwritten.
#-----------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)

project(graphslam_example)

# Basic MRPT infrastructure
find_package(mrpt_common REQUIRED)

# Dependencies
find_package(mrpt_graphslam REQUIRED)
find_package(mrpt_gui REQUIRED)


# Define the executable target:
mrpt_add_executable(
  TARGET graphslam_example
  SOURCES main.cpp
  LINK_LIBRARIES mrpt::mrpt_graphslam mrpt::mrpt_gui 
)
