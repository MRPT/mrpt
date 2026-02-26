#-----------------------------------------------------------------------------------------------
# CMake file for MRPT app
#-----------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)

project(ro-localization)

# Basic MRPT infrastructure
find_package(mrpt_common REQUIRED)

# Dependencies
find_package(mrpt_slam REQUIRED)
find_package(mrpt_gui REQUIRED)

# Define the executable target:
mrpt_add_executable(
  TARGET ${PROJECT_NAME}
  SOURCES
    CPosePDFParticlesExtended.cpp
    CPosePDFParticlesExtended.h
    ro-localization_main.cpp
  LINK_LIBRARIES 
    mrpt::mrpt_slam
    mrpt::mrpt_gui
)
