# ------------------------------------------------------------------------------
#        Mobile Robot Programming Toolkit (MRPT)
#
# Copyright (c) 2005-2026, Jose Luis Blanco-Claraco, contributors (see Git history)
# All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
# ------------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.16)

# Tell CMake we'll use C++ for use in its tests/flags
project(mrpt_typemeta LANGUAGES C CXX)

# MRPT CMake scripts: "mrpt_xxx()"
find_package(mrpt_common REQUIRED)

# define lib:
  set(LIB_SOURCES
  tests/typename_unittest.cpp
  tests/enumtype_unittest.cpp
  tests/static_string_unittest.cpp
  tests/xassert_unittest.cpp
)

set(LIB_PUBLIC_HEADERS
  include/mrpt/typemeta/static_string.h
  include/mrpt/typemeta/xassert.h
  include/mrpt/typemeta/TTypeName.h
  include/mrpt/typemeta/TEnumType.h
  include/mrpt/typemeta/num_to_string.h
  include/mrpt/typemeta/TTypeName_stl.h
)

mrpt_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES ${LIB_SOURCES} ${LIB_PUBLIC_HEADERS}
  HEADERS_ONLY_LIBRARY
#  PUBLIC_LINK_LIBRARIES
#   xxx
#  PRIVATE_LINK_LIBRARIES
#  CMAKE_DEPENDENCIES
#   xxx
)

