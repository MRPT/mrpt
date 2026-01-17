cmake_minimum_required(VERSION 3.16)
project(mrpt_vision_lgpl LANGUAGES C CXX)

find_package(mrpt_vision REQUIRED)

# define lib:
set(LIB_SOURCES "")
set(LIB_PUBLIC_HEADERS "")

if(MRPT_ALLOW_LGPLV3)
    set(LIB_SOURCES
      src/sift.cpp
      src/surf.cpp
    )
    set(LIB_PUBLIC_HEADERS
      include/mrpt/vision/sift.h
      include/mrpt/vision/surf.h
    )
    
    mrpt_add_library(
      TARGET ${PROJECT_NAME}
      SOURCES ${LIB_SOURCES} ${LIB_PUBLIC_HEADERS}
      PUBLIC_LINK_LIBRARIES
        mrpt::mrpt_vision
      CMAKE_DEPENDENCIES
        mrpt_vision
    )
endif()
