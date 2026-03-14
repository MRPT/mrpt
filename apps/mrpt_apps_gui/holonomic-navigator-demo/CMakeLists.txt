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
find_package(wxWidgets REQUIRED COMPONENTS gl core base adv)
find_package(OpenGL REQUIRED)

# Define the executable target:
mrpt_add_executable(
  TARGET ${PROJECT_NAME}
  SOURCES
    holonomic_navigator_demoApp.cpp
    holonomic_navigator_demoApp.h
    holonomic_navigator_demoMain.cpp
    holonomic_navigator_demoMain.h
    ../wx-common/MyGLCanvas.cpp
    ../wx-common/MyGLCanvas.h
    ../wxThings/src/toggle.cpp
  LINK_LIBRARIES
    mrpt::mrpt_nav
    mrpt::mrpt_gui
    wxWidgets::wxWidgets
    OpenGL::GL
)

target_include_directories(${PROJECT_NAME} PRIVATE ../wxThings/ ../wx-common/)

#AppStartMenuLink(${PROJECT_NAME} "Holonomic navigation simulator")   # Add link to the Start menu (in Windows Installable packages)
