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
find_package(mrpt_poses REQUIRED)

# After find_package(mrpt_data), the following variable is defined:
#   mrpt_data_DATA_DIR  - Absolute path to the installed data root directory
find_package(mrpt_data REQUIRED)

# Define C++ macro with the data directory path
add_compile_definitions(MRPT_EXAMPLE_DATA_DIR="")

# Define the executable target:
mrpt_add_executable(
  TARGET topography_coordinate_conversion_example
  SOURCES main.cpp
  LINK_LIBRARIES mrpt::mrpt_topography mrpt::mrpt_poses 
)
