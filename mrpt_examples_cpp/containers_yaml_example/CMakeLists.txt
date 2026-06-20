#-----------------------------------------------------------------------------------------------
# CMake file for the MRPT example: containers_yaml_example
# Generated automatically - Manual changes may be overwritten.
#-----------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)

project(containers_yaml_example)

# Basic MRPT infrastructure
find_package(mrpt_common REQUIRED)

# Dependencies
find_package(mrpt_containers REQUIRED)
find_package(mrpt_system REQUIRED)


# Define the executable target:
mrpt_add_executable(
  TARGET containers_yaml_example
  SOURCES main.cpp
  LINK_LIBRARIES mrpt::mrpt_containers mrpt::mrpt_system 
)
