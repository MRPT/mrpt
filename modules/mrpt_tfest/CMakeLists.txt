# ------------------------------------------------------------------------------
#        Mobile Robot Programming Toolkit (MRPT)
#
# Copyright (c) 2005-2026, Jose Luis Blanco-Claraco, contributors (see Git history)
# All rights reserved.
# Released under BSD-3 license. See LICENSE file
# ------------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.16)

# Tell CMake we'll use C++ for use in its tests/flags
project(mrpt_tfest LANGUAGES C CXX)

# MRPT CMake scripts: "mrpt_xxx()"
find_package(mrpt_common REQUIRED)
find_package(mrpt_poses REQUIRED)

# define lib:
set(LIB_SRCS
  src/registerAllClasses.cpp
  src/se2_l2_internal.h
  src/se2_l2_ransac.cpp
  src/se2_l2.cpp
  src/se2_l2.SSE2.cpp
  src/se3_l2_ransac.cpp
  src/se3_l2.cpp
  src/TMatchingPair.cpp
  tests/se3_unittest.cpp
)

set(LIB_PUBLIC_HDRS
  include/mrpt/tfest.h
  include/mrpt/tfest/indiv-compat-decls.h
  include/mrpt/tfest/registerAllClasses.h
  include/mrpt/tfest/se2.h
  include/mrpt/tfest/se3.h
  include/mrpt/tfest/TMatchingPair.h
)

if (BUILD_TESTING)
  find_package(mrpt_io REQUIRED) # for unit tests
endif()

mrpt_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES ${LIB_SRCS} ${LIB_PUBLIC_HDRS}
  PUBLIC_LINK_LIBRARIES
    mrpt::mrpt_poses
#  PRIVATE_LINK_LIBRARIES
  CMAKE_DEPENDENCIES
    mrpt_poses
)

target_precompile_headers(${PROJECT_NAME} PRIVATE
    <mrpt/tfest/se3.h>
)

# Python wrapper:
mrpt_add_python_module(tfest
  python_bindings/${PROJECT_NAME}_py.cpp
)

