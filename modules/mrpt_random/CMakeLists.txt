# ------------------------------------------------------------------------------
#        Mobile Robot Programming Toolkit (MRPT)
#
# Copyright (c) 2005-2025, Jose Luis Blanco-Claraco, contributors (see Git history)
# All rights reserved.
# Released under BSD-3 license. See LICENSE file
# ------------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.16)

# Tell CMake we'll use C++ for use in its tests/flags
project(mrpt_random LANGUAGES C CXX)

# MRPT CMake scripts: "mrpt_xxx()"
find_package(mrpt_common REQUIRED)

# find dependencies:
#...

# define lib:
set(LIB_SRCS
  src/RandomGenerator.cpp
  src/random_unittest.cpp
  src/random-precomp.cpp
)

set(LIB_PUBLIC_HDRS
  include/mrpt/random.h
  include/mrpt/random/RandomGenerators.h
  include/mrpt/random/random_shuffle.h
  include/mrpt/random/portable_uniform_distribution.h
)

mrpt_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES ${LIB_SRCS} ${LIB_PUBLIC_HDRS}
#  PUBLIC_LINK_LIBRARIES
#   xxx
#  PRIVATE_LINK_LIBRARIES
#  CMAKE_DEPENDENCIES
#   xxx
)
