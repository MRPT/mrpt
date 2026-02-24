#-----------------------------------------------------------------------------------------------
# CMake file for the MRPT example: topography_coordinate_conversion_example
# Generated automatically - Manual changes may be overwritten.
#-----------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)

project(topography_coordinate_conversion_example)

# Basic MRPT infrastructure
find_package(mrpt_common REQUIRED)

# Dependencies
find_package(mrpt_topography REQUIRED)


# Define the executable target:
mrpt_add_executable(
  TARGET topography_coordinate_conversion_example
  SOURCES main.cpp
  LINK_LIBRARIES mrpt::mrpt_topography 
)
