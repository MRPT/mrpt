#-----------------------------------------------------------------------------------------------
# CMake file for MRPT app
#-----------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)

project(yaml2ini)

# Basic MRPT infrastructure
find_package(mrpt_common REQUIRED)

# Dependencies
find_package(mrpt_config REQUIRED)
find_package(mrpt_system REQUIRED)
find_package(mrpt_io REQUIRED)

# Define the executable target:
mrpt_add_executable(
  TARGET ${PROJECT_NAME}
  SOURCES yaml2ini_main.cpp
  LINK_LIBRARIES 
    mrpt::mrpt_config
    mrpt::mrpt_system
    mrpt::mrpt_io
)
