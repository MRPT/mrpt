

#-----------------------------------------------------------------------------------------------
# CMake file for MRPT app
#-----------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)

project(track-video-features)

# Basic MRPT infrastructure
find_package(mrpt_common REQUIRED)

# Dependencies
find_package(mrpt_vision REQUIRED)
find_package(mrpt_gui REQUIRED)
find_package(mrpt_hwdrivers REQUIRED)
find_package(mrpt_libapps_gui REQUIRED)

# Define the executable target:
mrpt_add_executable(
  TARGET ${PROJECT_NAME}
  SOURCES track-video-feats_main.cpp
  LINK_LIBRARIES
    mrpt::mrpt_vision
    mrpt::mrpt_gui
    mrpt::mrpt_hwdrivers
    mrpt::mrpt_libapps_gui
)
