#-----------------------------------------------------------------------------------------------
# CMake file for the MRPT example: hwdrivers_mynteye_icp
# Generated automatically - Manual changes may be overwritten.
#-----------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)

project(hwdrivers_mynteye_icp)

# Basic MRPT infrastructure
find_package(mrpt_common REQUIRED)

# Dependencies
find_package(mrpt_slam REQUIRED)
find_package(mrpt_hwdrivers REQUIRED)
find_package(mrpt_gui REQUIRED)


# Define the executable target:
mrpt_add_executable(
  TARGET hwdrivers_mynteye_icp
  SOURCES main.cpp
  LINK_LIBRARIES mrpt::mrpt_slam mrpt::mrpt_hwdrivers mrpt::mrpt_gui 
)
