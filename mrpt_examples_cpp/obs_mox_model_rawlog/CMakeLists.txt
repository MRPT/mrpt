#-----------------------------------------------------------------------------------------------
# CMake file for the MRPT example: obs_mox_model_rawlog
# Generated automatically - Manual changes may be overwritten.
#-----------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)

project(obs_mox_model_rawlog)

# Basic MRPT infrastructure
find_package(mrpt_common REQUIRED)

# Dependencies
find_package(mrpt_obs REQUIRED)


# Define the executable target:
mrpt_add_executable(
  TARGET obs_mox_model_rawlog
  SOURCES main.cpp
  LINK_LIBRARIES mrpt::mrpt_obs 
)
