#-----------------------------------------------------------------------------------------------
# CMake file for the MRPT example: serialization_stl
# Generated automatically - Manual changes may be overwritten.
#-----------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)

project(serialization_stl)

# Basic MRPT infrastructure
find_package(mrpt_common REQUIRED)

# Dependencies
find_package(mrpt_serialization REQUIRED)
find_package(mrpt_io REQUIRED)
find_package(mrpt_poses REQUIRED)


# Define the executable target:
mrpt_add_executable(
  TARGET serialization_stl
  SOURCES main.cpp
  LINK_LIBRARIES mrpt::mrpt_serialization mrpt::mrpt_io mrpt::mrpt_poses 
)
