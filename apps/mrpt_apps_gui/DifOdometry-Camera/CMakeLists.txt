

#-----------------------------------------------------------------------------------------------
# CMake file for MRPT app
#-----------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)

project(DifOdometry-Camera)

# Basic MRPT infrastructure
find_package(mrpt_common REQUIRED)

# Dependencies
find_package(mrpt_gui REQUIRED)
find_package(mrpt_opengl REQUIRED)
find_package(mrpt_vision REQUIRED)
find_package(mrpt_hwdrivers REQUIRED)

# Define the executable target:
mrpt_add_executable(
  TARGET ${PROJECT_NAME}
  SOURCES
    DifOdometry_Camera_main.cpp
    DifOdometry_Camera.cpp
    DifOdometry_Camera.h
  LINK_LIBRARIES
    mrpt::mrpt_gui
    mrpt::mrpt_opengl
    mrpt::mrpt_vision
    mrpt::mrpt_hwdrivers
)
