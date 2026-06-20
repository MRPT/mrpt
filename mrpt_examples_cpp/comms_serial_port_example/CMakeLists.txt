#-----------------------------------------------------------------------------------------------
# CMake file for the MRPT example: comms_serial_port_example
# Generated automatically - Manual changes may be overwritten.
#-----------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)

project(comms_serial_port_example)

# Basic MRPT infrastructure
find_package(mrpt_common REQUIRED)

# Dependencies
find_package(mrpt_comms REQUIRED)
find_package(mrpt_serialization REQUIRED)
find_package(mrpt_poses REQUIRED)


# Define the executable target:
mrpt_add_executable(
  TARGET comms_serial_port_example
  SOURCES main.cpp
  LINK_LIBRARIES mrpt::mrpt_comms mrpt::mrpt_serialization mrpt::mrpt_poses 
)
