

#-----------------------------------------------------------------------------------------------
# CMake file for MRPT app
#-----------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)

project(SceneViewer3D)

# Basic MRPT infrastructure
find_package(mrpt_common REQUIRED)

# Dependencies
find_package(mrpt_opengl REQUIRED)
find_package(mrpt_gui REQUIRED)
find_package(mrpt_maps REQUIRED)

# Define the executable target:
mrpt_add_executable(
  TARGET ${PROJECT_NAME}
  SOURCES
    _DSceneViewerApp.cpp
    _DSceneViewerApp.h
    _DSceneViewerMain.cpp
    _DSceneViewerMain.h
    CDialogOptions.cpp
    CDialogOptions.h
    CDlgCamTracking.cpp
    CDlgCamTracking.h
    CDlgPLYOptions.cpp
    CDlgPLYOptions.h
  LINK_LIBRARIES
    mrpt::mrpt_opengl
    mrpt::mrpt_gui
    mrpt::mrpt_maps
)

#AppStartMenuLink(${PROJECT_NAME} "Scene Viewer 3D")   # Add link to the Start menu (in Windows Installable packages)

