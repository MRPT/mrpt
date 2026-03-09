#-----------------------------------------------------------------------------------------------
# CMake file for the MRPT example: opengl_custom_shaders_demo
# Generated automatically - Manual changes may be overwritten.
#-----------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)

project(opengl_custom_shaders_demo)

# Basic MRPT infrastructure
find_package(mrpt_common REQUIRED)

# Dependencies
find_package(mrpt_gui REQUIRED)


# Define the executable target:
mrpt_add_executable(
  TARGET opengl_custom_shaders_demo
  SOURCES main.cpp
  LINK_LIBRARIES mrpt::mrpt_gui
)
