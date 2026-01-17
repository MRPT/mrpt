# Lists of directories with source files:
#  See "DeclareMRPTLib.cmake" for explanations
# -------------------------------------------------
cmake_minimum_required(VERSION 3.16)
project(mrpt_nav LANGUAGES C CXX)

find_package(mrpt_maps REQUIRED)
find_package(mrpt_kinematics REQUIRED)

# define lib:
set(LIB_SOURCES
  src/TPTGAllowedBackStop.cpp
)

set(LIB_PUBLIC_HEADERS
  include/mrpt/nav/TPTGAllowedBackStop.h
)

mrpt_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES ${LIB_SOURCES} ${LIB_PUBLIC_HEADERS}
  PUBLIC_LINK_LIBRARIES
    mrpt::mrpt_maps
    mrpt::mrpt_kinematics
  CMAKE_DEPENDENCIES
    mrpt_maps
    mrpt_kinematics
)
