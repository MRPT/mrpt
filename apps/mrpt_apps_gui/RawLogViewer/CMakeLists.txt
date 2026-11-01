#-----------------------------------------------------------------------------------------------
# CMake file for MRPT app
#-----------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)

project(RawLogViewer)

# Basic MRPT infrastructure
find_package(mrpt_common REQUIRED)

# Dependencies
find_package(mrpt_libapps_cli REQUIRED)
find_package(mrpt_topography REQUIRED)
find_package(mrpt_imgui QUIET)
if(NOT mrpt_imgui_FOUND OR NOT TARGET mrpt_apps::imgui_common)
  message(STATUS "mrpt_imgui not found (built without GLFW?): skipping ${PROJECT_NAME}")
  return()
endif()
find_package(CLI11 REQUIRED)

set(RLV_DOC_SOURCES
  RawlogDocument.cpp
  RawlogDocument.h
)

mrpt_add_executable(
  TARGET ${PROJECT_NAME}
  SOURCES
    ${RLV_DOC_SOURCES}
    RawLogViewer_main.cpp
    RawLogViewerApp.cpp
    RawLogViewerApp.h
    RawLogViewerApp_common.h
    RawLogViewerApp_panels.cpp
  LINK_LIBRARIES
    mrpt_apps::imgui_common
    mrpt::mrpt_libapps_cli
    mrpt::mrpt_topography
    CLI11::CLI11
)

mrpt_add_test(
  TARGET test_RawLogViewer
  SOURCES
    ${RLV_DOC_SOURCES}
    tests/RawlogDocument_unittest.cpp
  LINK_LIBRARIES
    mrpt::mrpt_libapps_cli
    mrpt::mrpt_maps
)
if(TARGET test_RawLogViewer)
  target_include_directories(test_RawLogViewer PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
endif()
