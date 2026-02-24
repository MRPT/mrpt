#-----------------------------------------------------------------------------------------------
# CMake file for the MRPT example: maps_gmrf_map_example
# Generated automatically - Manual changes may be overwritten.
#-----------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)

project(maps_gmrf_map_example)

# Basic MRPT infrastructure
find_package(mrpt_common REQUIRED)

# Dependencies
find_package(mrpt_maps REQUIRED)
find_package(mrpt_gui REQUIRED)


# Define the executable target:
mrpt_add_executable(
  TARGET maps_gmrf_map_example
  SOURCES main.cpp
  LINK_LIBRARIES mrpt::mrpt_maps mrpt::mrpt_gui 
)
