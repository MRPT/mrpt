
cmake_minimum_required(VERSION 3.16)
project(mrpt_vision LANGUAGES C CXX)

find_package(mrpt_obs REQUIRED)

# define lib:
set(LIB_SOURCES
  src/CFeatureExtraction.cpp
  src/CFeatureList.cpp
)

set(LIB_PUBLIC_HEADERS
  include/mrpt/vision/CFeatureExtraction.h
  include/mrpt/vision/CFeatureList.h
)

mrpt_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES ${LIB_SOURCES} ${LIB_PUBLIC_HEADERS}
  PUBLIC_LINK_LIBRARIES
    mrpt::mrpt_obs
  CMAKE_DEPENDENCIES
    mrpt_obs
)
