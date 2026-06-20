#-----------------------------------------------------------------------------------------------
# CMake file for the MRPT example: opengl_offscreen_render_example
# Generated automatically - Manual changes may be overwritten.
#-----------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)

project(opengl_offscreen_render_example)

# Basic MRPT infrastructure
find_package(mrpt_common REQUIRED)

# Dependencies
find_package(mrpt_system REQUIRED)
find_package(mrpt_math REQUIRED)
find_package(mrpt_gui REQUIRED)


# Define the executable target:
mrpt_add_executable(
  TARGET opengl_offscreen_render_example
  SOURCES main.cpp
  LINK_LIBRARIES mrpt::mrpt_system mrpt::mrpt_math mrpt::mrpt_gui 
)
