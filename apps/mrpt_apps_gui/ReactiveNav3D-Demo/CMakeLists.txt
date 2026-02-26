

#-----------------------------------------------------------------------------------------------
# CMake file for MRPT app
#-----------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)

project(ReactiveNav3D-Demo)

# Basic MRPT infrastructure
find_package(mrpt_common REQUIRED)

# Dependencies
find_package(mrpt_gui REQUIRED)
find_package(mrpt_slam REQUIRED)
find_package(mrpt_nav REQUIRED)

# Define the executable target:
mrpt_add_executable(
  TARGET ${PROJECT_NAME}
  SOURCES
    ReactiveNav3D_demo_main.cpp
    ReactiveNav3D_demo.h
  LINK_LIBRARIES
    mrpt::mrpt_gui
    mrpt::mrpt_slam
    mrpt::mrpt_nav
)



#find_package( MRPT REQUIRED base gui opengl slam maps reactivenav)
