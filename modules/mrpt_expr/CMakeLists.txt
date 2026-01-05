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
  tests/CRuntimeCompiledExpression_unittest.cpp
)

# Use system exprtk library?
# Find the /usr/include/exprtk.hpp file:
find_path(
  EXPRTK_INCLUDE_DIR
  NAMES exprtk.hpp
  PATHS /usr/include /usr/local/include
  NO_DEFAULT_PATH
)
if(EXPRTK_INCLUDE_DIR)
  message(STATUS "Using system exprtk.hpp from: ${EXPRTK_INCLUDE_DIR}")
  set(MRPT_USE_SYSTEM_EXPRTK ON)
else()
  message(STATUS "System exprtk.hpp not found, using bundled version.")
  set(MRPT_USE_SYSTEM_EXPRTK OFF)
endif()

set(LIB_PUBLIC_HDRS
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

if (MRPT_USE_SYSTEM_EXPRTK)
  target_compile_definitions(${PROJECT_NAME} PRIVATE MRPT_USE_SYSTEM_EXPRTK)
  # Add include path as system path to avoid warnings from exprtk.hpp:
  target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE ${EXPRTK_INCLUDE_DIR})
else()
  # Add include path as system path: src/3rdparty/exprtk:
  target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/3rdparty/exprtk
  )
endif()

# Don't export ALL symbols for the huge exprtk lib
set_target_properties(${PROJECT_NAME} PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS 0)
# Minimize debug info for this module:
mrpt_reduced_debug_symbols(${PROJECT_NAME})
