#-----------------------------------------------------------------------------------------------
# CMake file for the MRPT example: poses_unscented_transform_example
# Generated automatically - Manual changes may be overwritten.
#-----------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)

project(poses_unscented_transform_example)

# Basic MRPT infrastructure
find_package(mrpt_common REQUIRED)

# Dependencies
find_package(mrpt_poses REQUIRED)
find_package(mrpt_io REQUIRED)
find_package(mrpt_gui REQUIRED)


# Define the executable target:
mrpt_add_executable(
  TARGET poses_unscented_transform_example
  SOURCES main.cpp
  LINK_LIBRARIES mrpt::mrpt_poses mrpt::mrpt_io mrpt::mrpt_gui 
)
