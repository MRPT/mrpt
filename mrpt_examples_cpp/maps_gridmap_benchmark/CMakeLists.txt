#-----------------------------------------------------------------------------------------------
# CMake file for the MRPT example: maps_gridmap_benchmark
# Generated automatically - Manual changes may be overwritten.
#-----------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)

project(maps_gridmap_benchmark)

# Basic MRPT infrastructure
find_package(mrpt_common REQUIRED)

# Dependencies
find_package(mrpt_slam REQUIRED)
find_package(mrpt_gui REQUIRED)


# Define the executable target:
mrpt_add_executable(
  TARGET maps_gridmap_benchmark
  SOURCES main.cpp
  LINK_LIBRARIES mrpt::mrpt_slam mrpt::mrpt_gui 
)
