#-----------------------------------------------------------------------------------------------
# CMake file for the MRPT example: rtti_example1
# Generated automatically - Manual changes may be overwritten.
#-----------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)

project(rtti_example1)

# Basic MRPT infrastructure
find_package(mrpt_common REQUIRED)

# Dependencies
find_package(mrpt_rtti REQUIRED)


# Define the executable target:
mrpt_add_executable(
  TARGET rtti_example1
  SOURCES main.cpp
  LINK_LIBRARIES mrpt::mrpt_rtti 
)
