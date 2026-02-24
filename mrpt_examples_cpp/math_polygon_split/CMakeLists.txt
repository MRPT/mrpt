#-----------------------------------------------------------------------------------------------
# CMake file for the MRPT example: math_polygon_split
# Generated automatically - Manual changes may be overwritten.
#-----------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)

project(math_polygon_split)

# Basic MRPT infrastructure
find_package(mrpt_common REQUIRED)

# Dependencies
find_package(mrpt_math REQUIRED)
find_package(mrpt_random REQUIRED)
find_package(mrpt_gui REQUIRED)
find_package(mrpt_maps REQUIRED)


# Define the executable target:
mrpt_add_executable(
  TARGET math_polygon_split
  SOURCES main.cpp
  LINK_LIBRARIES mrpt::mrpt_math mrpt::mrpt_random mrpt::mrpt_gui mrpt::mrpt_maps 
)
