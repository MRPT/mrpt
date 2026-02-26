#-----------------------------------------------------------------------------------------------
# CMake file for MRPT app
#-----------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)

project(2d-slam-demo)

# Basic MRPT infrastructure
find_package(mrpt_common REQUIRED)

# Dependencies
find_package(mrpt_gui REQUIRED)
find_package(mrpt_slam REQUIRED)
find_package(CLI11 REQUIRED)
find_package(wxWidgets REQUIRED)

# Define the executable target:
mrpt_add_executable(
  TARGET ${PROJECT_NAME}
  SOURCES 
    slamdemoApp.cpp
    slamdemoApp.h
    slamdemoMain.cpp
    slamdemoMain.h
    batch_experiments.cpp
    CDlgParams.cpp
    CDlgParams.h
    CLogView.cpp
    CLogView.h
  LINK_LIBRARIES
    mrpt::mrpt_gui
    mrpt::mrpt_slam
    CLI11::CLI11
    wxWidgets::wxWidgets
)


#AppStartMenuLink(${PROJECT_NAME} "2D KF-SLAM Simulator")   # Add link to the Start menu (in Windows Installable packages)
