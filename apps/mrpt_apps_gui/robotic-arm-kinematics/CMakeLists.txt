#-----------------------------------------------------------------------------------------------
# CMake file for MRPT app
#-----------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)

project(robotic-arm-kinematics)

# Basic MRPT infrastructure
find_package(mrpt_common REQUIRED)

# Dependencies
find_package(mrpt_kinematics REQUIRED)
find_package(mrpt_gui REQUIRED)

# Define the executable target:
mrpt_add_executable(
  TARGET ${PROJECT_NAME}
  SOURCES
    robotic_arm_kinematicsApp.cpp
    robotic_arm_kinematicsApp.h
    robotic_arm_kinematicsMain.cpp
    robotic_arm_kinematicsMain.h
    PanelDOF.cpp
    PanelDOF.h
    CAboutBox.cpp
    CAboutBox.h
    MyGLCanvas.cpp
    MyGLCanvas.h
  LINK_LIBRARIES
    mrpt::mrpt_kinematics
    mrpt::mrpt_gui
)


#AppStartMenuLink(${PROJECT_NAME} "GUI for Denavit-Hartenberg parameters robot arm design")   # Add link to the Start menu (in Windows Installable packages)
