#-----------------------------------------------------------------------------------------------
# CMake file for the MRPT example: containers_params_by_name
# Generated automatically - Manual changes may be overwritten.
#-----------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)

project(containers_params_by_name)

# Basic MRPT infrastructure
find_package(mrpt_common REQUIRED)

# Dependencies
find_package(mrpt_containers REQUIRED)
find_package(mrpt_system REQUIRED)


# Define the executable target:
mrpt_add_executable(
  TARGET containers_params_by_name
  SOURCES main.cpp
  LINK_LIBRARIES mrpt::mrpt_containers mrpt::mrpt_system 
)
