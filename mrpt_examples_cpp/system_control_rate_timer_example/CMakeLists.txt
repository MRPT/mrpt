#-----------------------------------------------------------------------------------------------
# CMake file for the MRPT example: system_control_rate_timer_example
# Generated automatically - Manual changes may be overwritten.
#-----------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)

project(system_control_rate_timer_example)

# Basic MRPT infrastructure
find_package(mrpt_common REQUIRED)

# Dependencies
find_package(mrpt_system REQUIRED)
find_package(mrpt_math REQUIRED)
find_package(mrpt_gui REQUIRED)
find_package(CLI11 REQUIRED)


# Define the executable target:
mrpt_add_executable(
  TARGET system_control_rate_timer_example
  SOURCES main.cpp
  LINK_LIBRARIES mrpt::mrpt_system mrpt::mrpt_math mrpt::mrpt_gui CLI11::CLI11 
)
