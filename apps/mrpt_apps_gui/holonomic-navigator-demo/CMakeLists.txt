#-----------------------------------------------------------------------------------------------
# CMake file for MRPT app
#-----------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)

project(holonomic-navigator-demo)

# Basic MRPT infrastructure
find_package(mrpt_common REQUIRED)

# Dependencies
find_package(mrpt_nav REQUIRED)
find_package(mrpt_gui REQUIRED)

# Define the executable target:
mrpt_add_executable(
  TARGET ${PROJECT_NAME}
  SOURCES
    holonomic_navigator_demoApp.cpp
    holonomic_navigator_demoApp.h
    holonomic_navigator_demoMain.cpp
    holonomic_navigator_demoMain.h
    MyGLCanvas.cpp
    MyGLCanvas.h
  LINK_LIBRARIES
    mrpt::mrpt_nav
    mrpt::mrpt_gui
)

#AppStartMenuLink(${PROJECT_NAME} "Holonomic navigation simulator")   # Add link to the Start menu (in Windows Installable packages)
