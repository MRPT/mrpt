#-----------------------------------------------------------------------------------------------
# CMake file for the MRPT example: img_gauss_filtering_example
# Generated automatically - Manual changes may be overwritten.
#-----------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)

project(img_gauss_filtering_example)

# Basic MRPT infrastructure
find_package(mrpt_common REQUIRED)

# Dependencies
find_package(mrpt_gui REQUIRED)
find_package(mrpt_maps REQUIRED)


# Define the executable target:
mrpt_add_executable(
  TARGET img_gauss_filtering_example
  SOURCES main.cpp
  LINK_LIBRARIES mrpt::mrpt_gui mrpt::mrpt_maps 
)
