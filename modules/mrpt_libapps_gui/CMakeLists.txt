cmake_minimum_required(VERSION 3.16)
project(mrpt_libapps_gui LANGUAGES C CXX)

find_package(mrpt_libapps_cli REQUIRED)
find_package(mrpt_gui REQUIRED)

# define lib:
set(LIB_SOURCES
  src/CProgressDialog.cpp
  src/SimpleMainWindow.cpp
)

set(LIB_PUBLIC_HEADERS
  include/mrpt/apps/CProgressDialog.h
  include/mrpt/apps/SimpleMainWindow.h
)

mrpt_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES ${LIB_SOURCES} ${LIB_PUBLIC_HEADERS}
  PUBLIC_LINK_LIBRARIES
    mrpt::mrpt_libapps_cli
    mrpt::mrpt_gui
  CMAKE_DEPENDENCIES
    mrpt_libapps_cli
    mrpt_gui
)

if(NOT BUILD_mrpt-apps-gui)
  return()
endif()

target_link_libraries(${PROJECT_NAME} PRIVATE Threads::Threads)
