# ------------------------------------------------------------------------------
#        Mobile Robot Programming Toolkit (MRPT)
#
# Copyright (c) 2005-2025, Jose Luis Blanco-Claraco, contributors (see Git history)
# All rights reserved.
# Released under BSD-3 license. See LICENSE file
# ------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)

# Tell CMake we'll use C++ for use in its tests/flags
project(mrpt_expr LANGUAGES C CXX)

# MRPT CMake scripts: "mrpt_xxx()"
find_package(mrpt_common REQUIRED)
find_package(mrpt_system REQUIRED)

# define lib:
set(LIB_SRCS
  src/CRuntimeCompiledExpression.cpp
  src/CRuntimeCompiledExpression_unittest.cpp
)

set(LIB_PUBLIC_HDRS
  include/mrpt/3rdparty/exprtk.hpp
  include/mrpt/expr/mrpt-expr_export.h
  include/mrpt/expr/CRuntimeCompiledExpression.h
)

mrpt_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES ${LIB_SRCS} ${LIB_PUBLIC_HDRS}
  PUBLIC_LINK_LIBRARIES
    mrpt::mrpt_system
#  PRIVATE_LINK_LIBRARIES
  CMAKE_DEPENDENCIES
    mrpt_system
)

# Don't export ALL symbols for the huge exprtk lib
set_target_properties(${PROJECT_NAME} PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS 0)
# Minimize debug info for this module:
mrpt_reduced_debug_symbols(${PROJECT_NAME})
