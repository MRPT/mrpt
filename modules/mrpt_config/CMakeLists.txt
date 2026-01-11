# ------------------------------------------------------------------------------
#        Mobile Robot Programming Toolkit (MRPT)
#
# Copyright (c) 2005-2026, Jose Luis Blanco-Claraco, contributors (see Git history)
# All rights reserved.
# Released under BSD-3 license. See LICENSE file
# ------------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.16)

# Tell CMake we'll use C++ for use in its tests/flags
project(mrpt_config LANGUAGES C CXX)

# MRPT CMake scripts: "mrpt_xxx()"
find_package(mrpt_common REQUIRED)
find_package(mrpt_expr REQUIRED)

include(./cmake/script_simpleini.cmake REQUIRED)   # SimpleINI lib

# define lib:
set(LIB_SOURCES
  src/config_parser.cpp
  src/CConfigFilePrefixer.cpp
  src/CConfigFileBase.cpp
  src/CLoadableOptions.cpp
  src/CConfigFileMemory.cpp
  src/CConfigFile.cpp
  tests/yaml2config_unittest.cpp
  tests/CConfigFileMemory_unittest.cpp
)

set(LIB_PUBLIC_HEADERS
  include/mrpt/config/config_parser.h
  include/mrpt/config/CConfigFileBase.h
  include/mrpt/config/registerAllClasses.h
  include/mrpt/config/CConfigFile.h
  include/mrpt/config/CLoadableOptions.h
  include/mrpt/config/CConfigFileMemory.h
  include/mrpt/config/CConfigFilePrefixer.h
)

mrpt_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES ${LIB_SOURCES} ${LIB_PUBLIC_HEADERS}
  PUBLIC_LINK_LIBRARIES
    mrpt::mrpt_expr
#  PRIVATE_LINK_LIBRARIES
  CMAKE_DEPENDENCIES
    mrpt_expr
  # Other imported targets:
)

target_include_directories(${PROJECT_NAME} PRIVATE ${SIMPLEINI_INCLUDE_DIRS})
if (SIMPLEINI_LIBRARY)
  target_link_libraries(${PROJECT_NAME} PRIVATE ${SIMPLEINI_LIBRARY})
endif()


# Python wrapper:
mrpt_add_python_module(config
  python_bindings/${PROJECT_NAME}_py.cpp
)

