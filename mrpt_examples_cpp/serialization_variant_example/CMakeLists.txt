#-----------------------------------------------------------------------------------------------
# CMake file for the MRPT example: serialization_variant_example
# Generated automatically - Manual changes may be overwritten.
#-----------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)

project(serialization_variant_example)

# Basic MRPT infrastructure
find_package(mrpt_common REQUIRED)

# Dependencies
find_package(mrpt_serialization REQUIRED)
find_package(mrpt_io REQUIRED)
find_package(mrpt_poses REQUIRED)


# Define the executable target:
mrpt_add_executable(
  TARGET serialization_variant_example
  SOURCES main.cpp
  LINK_LIBRARIES mrpt::mrpt_serialization mrpt::mrpt_io mrpt::mrpt_poses 
)
