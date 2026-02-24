#-----------------------------------------------------------------------------------------------
# CMake file for the MRPT example: system_progress_bar
# Generated automatically - Manual changes may be overwritten.
#-----------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)

project(system_progress_bar)

# Basic MRPT infrastructure
find_package(mrpt_common REQUIRED)

# Dependencies
find_package(mrpt_system REQUIRED)
find_package(mrpt_poses REQUIRED)
find_package(mrpt_io REQUIRED)


# Define the executable target:
mrpt_add_executable(
  TARGET system_progress_bar
  SOURCES main.cpp
  LINK_LIBRARIES mrpt::mrpt_system mrpt::mrpt_poses mrpt::mrpt_io 
)
