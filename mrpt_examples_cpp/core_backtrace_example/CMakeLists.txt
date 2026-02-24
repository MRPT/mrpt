#-----------------------------------------------------------------------------------------------
# CMake file for the MRPT example: core_backtrace_example
# Generated automatically - Manual changes may be overwritten.
#-----------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)

project(core_backtrace_example)

# Basic MRPT infrastructure
find_package(mrpt_common REQUIRED)

# Dependencies
find_package(mrpt_core REQUIRED)


# Define the executable target:
mrpt_add_executable(
  TARGET core_backtrace_example
  SOURCES main.cpp
  LINK_LIBRARIES mrpt::mrpt_core 
)
