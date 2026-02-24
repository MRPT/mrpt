#-----------------------------------------------------------------------------------------------
# CMake file for the MRPT example: opengl_ray_trace_example
# Generated automatically - Manual changes may be overwritten.
#-----------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)

project(opengl_ray_trace_example)

# Basic MRPT infrastructure
find_package(mrpt_common REQUIRED)

# Dependencies
find_package(mrpt_gui REQUIRED)
find_package(mrpt_maps REQUIRED)


# Define the executable target:
mrpt_add_executable(
  TARGET opengl_ray_trace_example
  SOURCES main.cpp
  LINK_LIBRARIES mrpt::mrpt_gui mrpt::mrpt_maps 
)
