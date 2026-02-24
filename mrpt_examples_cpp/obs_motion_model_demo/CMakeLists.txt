#-----------------------------------------------------------------------------------------------
# CMake file for the MRPT example: obs_motion_model_demo
# Generated automatically - Manual changes may be overwritten.
#-----------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)

project(obs_motion_model_demo)

# Basic MRPT infrastructure
find_package(mrpt_common REQUIRED)

# Dependencies
find_package(mrpt_maps REQUIRED)
find_package(mrpt_gui REQUIRED)


# Define the executable target:
mrpt_add_executable(
  TARGET obs_motion_model_demo
  SOURCES main.cpp
  LINK_LIBRARIES mrpt::mrpt_maps mrpt::mrpt_gui 
)
