# ------------------------------------------------------------------------------
#        Mobile Robot Programming Toolkit (MRPT)
#
# Copyright (c) 2005-2025, Jose Luis Blanco-Claraco, contributors (see Git history)
# All rights reserved.
# Released under BSD-3 license. See LICENSE file
# ------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)

# Tell CMake we'll use C++ for use in its tests/flags
project(mrpt_system LANGUAGES C CXX)

# MRPT CMake scripts: "mrpt_xxx()"
find_package(mrpt_common REQUIRED)
find_package(mrpt_core REQUIRED)
find_package(mrpt_typemeta REQUIRED)

# define lib:
set(LIB_SRCS
  src/CListOfClasses.cpp
  src/CObject.cpp
  src/internal_class_registry.cpp
  src/rtti_unittest.cpp
)

set(LIB_PUBLIC_HDRS
  include/mrpt/rtti/CListOfClasses.h
  include/mrpt/rtti/registerAllClasses.h
  include/mrpt/rtti/CObject.h
)

mrpt_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES ${LIB_SRCS} ${LIB_PUBLIC_HDRS}
  PUBLIC_LINK_LIBRARIES
    mrpt::mrpt_core
    mrpt::mrpt_typemeta
#  PRIVATE_LINK_LIBRARIES
  CMAKE_DEPENDENCIES
    mrpt_core
    mrpt_typemeta
)
