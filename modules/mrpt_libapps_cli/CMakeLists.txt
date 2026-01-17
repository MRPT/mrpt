cmake_minimum_required(VERSION 3.16)
project(mrpt_libapps_cli LANGUAGES C CXX)

find_package(mrpt_hwdrivers REQUIRED)
find_package(mrpt_slam REQUIRED)
find_package(mrpt_topography REQUIRED)

# define lib:
set(LIB_SOURCES
  src/RawlogEditApp.cpp
  src/RawlogGrabberApp.cpp
  src/DataSourceRawlog.cpp
)

set(LIB_PUBLIC_HEADERS
  include/mrpt/apps/RawlogEditApp.h
  include/mrpt/apps/RawlogGrabberApp.h
)

mrpt_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES ${LIB_SOURCES} ${LIB_PUBLIC_HEADERS}
  PUBLIC_LINK_LIBRARIES
    mrpt::mrpt_hwdrivers
    mrpt::mrpt_slam
    mrpt::mrpt_topography
  CMAKE_DEPENDENCIES
    mrpt_hwdrivers
    mrpt_slam
    mrpt_topography
)

if(NOT BUILD_mrpt-apps-cli)
  return()
endif()

target_link_libraries(${PROJECT_NAME} PRIVATE Threads::Threads)
