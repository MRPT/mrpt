#-----------------------------------------------------------------------------------------------
# CMake file for the MRPT example: random_examples
# Generated automatically - Manual changes may be overwritten.
#-----------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)

project(random_examples)

# Basic MRPT infrastructure
find_package(mrpt_common REQUIRED)

# Dependencies
find_package(mrpt_random REQUIRED)
find_package(mrpt_gui REQUIRED)


# Define the executable target:
mrpt_add_executable(
  TARGET random_examples
  SOURCES main.cpp
  LINK_LIBRARIES mrpt::mrpt_random mrpt::mrpt_gui 
)
