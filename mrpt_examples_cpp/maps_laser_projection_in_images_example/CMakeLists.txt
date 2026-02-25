#-----------------------------------------------------------------------------------------------
# CMake file for the MRPT example: maps_laser_projection_in_images_example
# Generated automatically - Manual changes may be overwritten.
#-----------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)

project(maps_laser_projection_in_images_example)

# Basic MRPT infrastructure
find_package(mrpt_common REQUIRED)

# Dependencies
find_package(mrpt_maps REQUIRED)
find_package(mrpt_gui REQUIRED)

# After find_package(mrpt_data), the following variable is defined:
#   mrpt_data_DATA_DIR  - Absolute path to the installed data root directory
find_package(mrpt_data REQUIRED)

# Define C++ macro with the data directory path
add_compile_definitions(MRPT_EXAMPLE_DATA_DIR="")

# Define the executable target:
mrpt_add_executable(
  TARGET maps_laser_projection_in_images_example
  SOURCES main.cpp
  LINK_LIBRARIES mrpt::mrpt_maps mrpt::mrpt_gui 
)
