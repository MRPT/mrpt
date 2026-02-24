#-----------------------------------------------------------------------------------------------
# CMake file for the MRPT example: rgbd_dataset2rawlog
# Generated automatically - Manual changes may be overwritten.
#-----------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)

project(rgbd_dataset2rawlog)

# Basic MRPT infrastructure
find_package(mrpt_common REQUIRED)

# Dependencies
find_package(mrpt_obs REQUIRED)


# Define the executable target:
mrpt_add_executable(
  TARGET rgbd_dataset2rawlog
  SOURCES main.cpp
  LINK_LIBRARIES mrpt::mrpt_obs 
)
