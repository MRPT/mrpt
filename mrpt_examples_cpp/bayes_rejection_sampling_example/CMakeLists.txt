#-----------------------------------------------------------------------------------------------
# CMake file for the MRPT example: bayes_rejection_sampling_example
# Generated automatically - Manual changes may be overwritten.
#-----------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)

project(bayes_rejection_sampling_example)

# Basic MRPT infrastructure
find_package(mrpt_common REQUIRED)

# Dependencies
find_package(mrpt_bayes REQUIRED)
find_package(mrpt_obs REQUIRED)
find_package(mrpt_gui REQUIRED)


# Define the executable target:
mrpt_add_executable(
  TARGET bayes_rejection_sampling_example
  SOURCES main.cpp
  LINK_LIBRARIES mrpt::mrpt_bayes mrpt::mrpt_obs mrpt::mrpt_gui 
)
