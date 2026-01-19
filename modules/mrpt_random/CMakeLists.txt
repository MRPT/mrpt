# ------------------------------------------------------------------------------
#        Mobile Robot Programming Toolkit (MRPT)
#
# Copyright (c) 2005-2026, Jose Luis Blanco-Claraco, contributors (see Git history)
# All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
# ------------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.16)

# Tell CMake we'll use C++ for use in its tests/flags
project(mrpt_random LANGUAGES C CXX)

# MRPT CMake scripts: "mrpt_xxx()"
find_package(mrpt_common REQUIRED)

# find dependencies:
#...

# define lib:
set(LIB_SOURCES
  src/RandomGenerator.cpp
  tests/random_unittest.cpp
)

set(LIB_PUBLIC_HEADERS
  include/mrpt/random.h
  include/mrpt/random/RandomGenerators.h
  include/mrpt/random/random_shuffle.h
  include/mrpt/random/portable_uniform_distribution.h
)

mrpt_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES ${LIB_SOURCES} ${LIB_PUBLIC_HEADERS}
#  PUBLIC_LINK_LIBRARIES
#   xxx
#  PRIVATE_LINK_LIBRARIES
#  CMAKE_DEPENDENCIES
#   xxx
)
