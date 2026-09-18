# ------------------------------------------------------------------------------
#        Mobile Robot Programming Toolkit (MRPT)
#
# Copyright (c) 2005-2026, Jose Luis Blanco-Claraco, contributors (see Git history)
# All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
# ------------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.16)

# Tell CMake we'll use C++ for use in its tests/flags
project(mrpt_topography LANGUAGES C CXX)

# MRPT CMake scripts: "mrpt_xxx()"
find_package(mrpt_common REQUIRED)
find_package(mrpt_math REQUIRED)

# define lib:
set(LIB_SOURCES
  src/conversions.cpp
  src/registerAllClasses.cpp
)

set(LIB_UNIT_TEST_SOURCES
  tests/conversions_unittest.cpp
  tests/data_types_unittest.cpp
  tests/transforms_unittest.cpp
)

set(LIB_PUBLIC_HEADERS
  include/mrpt/topography/registerAllClasses.h
  include/mrpt/topography/conversions.h
  include/mrpt/topography/data_types.h
  include/mrpt/topography.h
)

mrpt_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES ${LIB_SOURCES} ${LIB_PUBLIC_HEADERS}
  UNIT_TEST_SOURCES ${LIB_UNIT_TEST_SOURCES}
  PUBLIC_LINK_LIBRARIES
    mrpt::mrpt_math
#  PRIVATE_LINK_LIBRARIES
  CMAKE_DEPENDENCIES
    mrpt_math
)

# Python wrapper:
mrpt_add_python_module(topography
  python_bindings/${PROJECT_NAME}_py.cpp
)

mrpt_add_python_binding_test(topography
  SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/tests/python_topography_test.py
)

