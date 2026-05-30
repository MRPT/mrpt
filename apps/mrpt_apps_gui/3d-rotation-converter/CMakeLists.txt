#-----------------------------------------------------------------------------------------------
# CMake file for MRPT app
#-----------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)

project(3d-rotation-converter)

# Basic MRPT infrastructure
find_package(mrpt_common REQUIRED)

# Dependencies
find_package(mrpt_gui REQUIRED)

# This app requires nanogui (part of mrpt_gui). Skip if not available.
if(NOT TARGET mrpt_nanogui AND NOT TARGET mrpt::mrpt_nanogui)
  message(STATUS "nanogui not available — skipping ${PROJECT_NAME}")
  return()
endif()

# Define the executable target:
mrpt_add_executable(
  TARGET ${PROJECT_NAME}
  SOURCES 3d-rotation-converter_main.cpp
  LINK_LIBRARIES
    mrpt::mrpt_gui
)
