
cmake_minimum_required(VERSION 3.16)
project(mrpt_vision_lgpl LANGUAGES C CXX)

find_package(mrpt_common REQUIRED)
find_package(mrpt_vision REQUIRED)

set(LIB_SOURCES
  src/ba_common.cpp
  src/ba_full.cpp
)

set(LIB_PUBLIC_HEADERS
  include/mrpt/vision/bundle_adjustment.h
  src/ba_internals.h
)

mrpt_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES ${LIB_SOURCES} ${LIB_PUBLIC_HEADERS}
  PUBLIC_LINK_LIBRARIES
    mrpt::mrpt_vision
  CMAKE_DEPENDENCIES
    mrpt_vision
)
