#-----------------------------------------------------------------------------------------------
# CMake file for MRPT app
#-----------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)

project(navlog-viewer)

# Basic MRPT infrastructure
find_package(mrpt_common REQUIRED)

# Dependencies
find_package(mrpt_gui REQUIRED)
find_package(mrpt_nav REQUIRED)
find_package(CLI11 REQUIRED)

# Define the executable target:
mrpt_add_executable(
  TARGET ${PROJECT_NAME}
  SOURCES
    main.cpp
    navlog-viewer-ui.cpp
    navlog-viewer-ui.h
  LINK_LIBRARIES
    mrpt::mrpt_gui
    mrpt::mrpt_nav
    CLI11::CLI11
)
