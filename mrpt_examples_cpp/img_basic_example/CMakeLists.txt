#-----------------------------------------------------------------------------------------------
# CMake file for the MRPT example: img_basic_example
# Generated automatically - Manual changes may be overwritten.
#-----------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)

project(img_basic_example)

# Basic MRPT infrastructure
find_package(mrpt_common REQUIRED)

# Dependencies
find_package(mrpt_gui REQUIRED)
find_package(mrpt_maps REQUIRED)

# After find_package(mrpt_data), the following variable is defined:
#   mrpt_data_DATA_DIR  - Absolute path to the installed data root directory
find_package(mrpt_data REQUIRED)

# Define C++ macro with the data directory path
add_compile_definitions(MRPT_EXAMPLE_DATA_DIR="")

# Define the executable target:
mrpt_add_executable(
  TARGET img_basic_example
  SOURCES main.cpp
  LINK_LIBRARIES mrpt::mrpt_gui mrpt::mrpt_maps 
)
