#-----------------------------------------------------------------------------------------------
# CMake file for MRPT app
#-----------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)

project(kinect-stereo-calib)

# Basic MRPT infrastructure
find_package(mrpt_common REQUIRED)

# Dependencies
find_package(mrpt_hwdrivers REQUIRED)
find_package(mrpt_gui REQUIRED)
find_package(mrpt_vision REQUIRED)
find_package(wxWidgets REQUIRED COMPONENTS gl core base adv)

# Define the executable target:
mrpt_add_executable(
  TARGET ${PROJECT_NAME}
  SOURCES
    kinect_calibrate_guiApp.cpp
    kinect_calibrate_guiApp.h
    kinect_calibrate_guiMain.cpp
    kinect_calibrate_guiMain.h
    MyGLCanvas.cpp
    MyGLCanvas.h
  LINK_LIBRARIES
    mrpt::mrpt_hwdrivers
    mrpt::mrpt_gui
    mrpt::mrpt_vision
    wxWidgets::wxWidgets
)

#AppStartMenuLink(${PROJECT_NAME} "Kinect (and generic stereo cams) calibration tool")   # Add link to the Start menu (in Windows Installable packages)

