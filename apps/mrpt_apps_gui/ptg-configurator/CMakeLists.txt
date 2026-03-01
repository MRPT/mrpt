#-----------------------------------------------------------------------------------------------
# CMake file for MRPT app
#-----------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)

project(ptg-configurator)

# Basic MRPT infrastructure
find_package(mrpt_common REQUIRED)

# Dependencies
find_package(mrpt_gui REQUIRED)
find_package(mrpt_nav REQUIRED)
find_package(wxWidgets REQUIRED COMPONENTS gl core base adv)

# Define the executable target:
mrpt_add_executable(
  TARGET ${PROJECT_NAME}
  SOURCES
    ptgConfiguratorApp.cpp
    ptgConfiguratorApp.h
    ptgConfiguratorMain.cpp
    ptgConfiguratorMain.h
    MyGLCanvas.cpp
    MyGLCanvas.h
  LINK_LIBRARIES
    mrpt::mrpt_gui
    mrpt::mrpt_nav
    wxWidgets::wxWidgets
)
