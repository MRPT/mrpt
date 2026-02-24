#-----------------------------------------------------------------------------------------------
# CMake file for the MRPT example: slam_range_only_localization_rej_sampling_example
# Generated automatically - Manual changes may be overwritten.
#-----------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)

project(slam_range_only_localization_rej_sampling_example)

# Basic MRPT infrastructure
find_package(mrpt_common REQUIRED)

# Dependencies
find_package(mrpt_slam REQUIRED)
find_package(mrpt_gui REQUIRED)


# Define the executable target:
mrpt_add_executable(
  TARGET slam_range_only_localization_rej_sampling_example
  SOURCES main.cpp
  LINK_LIBRARIES mrpt::mrpt_slam mrpt::mrpt_gui 
)
