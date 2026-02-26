

#-----------------------------------------------------------------------------------------------
# CMake file for MRPT app
#-----------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)

project(GridmapNavSimul)

# Basic MRPT infrastructure
find_package(mrpt_common REQUIRED)

# Dependencies
find_package(mrpt_maps REQUIRED)
find_package(mrpt_hwdrivers REQUIRED)
find_package(mrpt_kinematics REQUIRED)

# Define the executable target:
mrpt_add_executable(
  TARGET ${PROJECT_NAME}
  SOURCES
    gridmapSimulApp.cpp
    gridmapSimulApp.h
    gridmapSimulMain.cpp
    gridmapSimulMain.h
  LINK_LIBRARIES
    mrpt::mrpt_maps
    mrpt::mrpt_hwdrivers
    mrpt::mrpt_kinematics
)
