cmake_minimum_required(VERSION 3.16)

project(mrpt_graphslam LANGUAGES C CXX)

# Make sure the dependency package is available to CMake
find_package(mrpt_slam REQUIRED)

# define lib:
set(LIB_SOURCES
  src/TSlidingWindow.cpp
  src/CWindowObserver.cpp
  src/CEdgeCounter.cpp
)

set(LIB_PUBLIC_HEADERS
  include/mrpt/graphslam/CWindowManager.h
  include/mrpt/graphslam/CWindowObserver.h
  include/mrpt/graphslam/misc/TSlidingWindow.h
)

mrpt_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES ${LIB_SOURCES} ${LIB_PUBLIC_HEADERS}
  PUBLIC_LINK_LIBRARIES
    mrpt::mrpt_slam
  CMAKE_DEPENDENCIES
    mrpt_slam
)
