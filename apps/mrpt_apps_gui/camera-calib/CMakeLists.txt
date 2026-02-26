#-----------------------------------------------------------------------------------------------
# CMake file for MRPT app
#-----------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)

project(camera-calib)

# Basic MRPT infrastructure
find_package(mrpt_common REQUIRED)

# Dependencies
find_package(mrpt_vision REQUIRED)
find_package(mrpt_gui REQUIRED)
find_package(mrpt_hwdrivers REQUIRED)

# Define the executable target:
mrpt_add_executable(
  TARGET ${PROJECT_NAME}
  SOURCES
    camera_calib_guiApp.cpp
    camera_calib_guiApp.h
    camera_calib_guiMain.cpp
    camera_calib_guiMain.h
    CDlgCalibWizardOnline.cpp
    CDlgCalibWizardOnline.h
    CDlgPoseEst.cpp
    CDlgPoseEst.h
    MyGLCanvas.cpp
    MyGLCanvas.h
  LINK_LIBRARIES
    mrpt::mrpt_vision
    mrpt::mrpt_gui
    mrpt::mrpt_hwdrivers
)

#AppStartMenuLink(${PROJECT_NAME} "Camera calibration GUI")   # Add link to the Start menu (in Windows Installable packages)
