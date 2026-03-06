#-----------------------------------------------------------------------------------------------
# CMake file for MRPT app
#-----------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)

project(ReactiveNavigationDemo)

# Basic MRPT infrastructure
find_package(mrpt_common REQUIRED)

# Dependencies
find_package(mrpt_gui REQUIRED)
find_package(mrpt_nav REQUIRED)
find_package(mrpt_kinematics REQUIRED)
find_package(wxWidgets COMPONENTS core base gl adv QUIET)
if(NOT wxWidgets_FOUND)
  message(STATUS "wxWidgets not found — skipping ReactiveNavigationDemo")
  return()
endif()
include(${wxWidgets_USE_FILE})

# Define the executable target:
mrpt_add_executable(
  TARGET ${PROJECT_NAME}
  SOURCES
    reactive_navigator_demoApp.cpp
    reactive_navigator_demoApp.h
    reactive_navigator_demoMain.cpp
    reactive_navigator_demoMain.h
    DEFAULT_GRIDMAP_DATA.h
    MyGLCanvas.cpp
    MyGLCanvas.h
    ../wxThings/src/toggle.cpp
  LINK_LIBRARIES
    mrpt::mrpt_gui
    mrpt::mrpt_nav
    mrpt::mrpt_kinematics
    ${wxWidgets_LIBRARIES}
)

target_include_directories(${PROJECT_NAME} PRIVATE ../wxThings)

#AppStartMenuLink(${PROJECT_NAME} "Reactive navigation demo")   # Add link to the Start menu (in Windows Installable packages)

