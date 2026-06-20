#-----------------------------------------------------------------------------------------------
# CMake file for the MRPT example: topography_gps_coords_example
# Generated automatically - Manual changes may be overwritten.
#-----------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)

project(topography_gps_coords_example)

# Basic MRPT infrastructure
find_package(mrpt_common REQUIRED)

# Dependencies
find_package(mrpt_topography REQUIRED)
find_package(mrpt_poses REQUIRED)

# This example uses Eigen directly (calls MRPT matrix methods that require
# <Eigen/Dense> in the calling translation unit):
find_package(Eigen3 REQUIRED)

# Define the executable target:
mrpt_add_executable(
  TARGET topography_gps_coords_example
  SOURCES main.cpp
  LINK_LIBRARIES mrpt::mrpt_topography mrpt::mrpt_poses Eigen3::Eigen
)
