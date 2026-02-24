# MRPT Examples C++ - CMakeLists.txt
#/*                    _
#                     | |    Mobile Robot Programming Toolkit (MRPT)
# _ __ ___  _ __ _ __ | |_
#| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
#| | | | | | |  | |_) | |_
#|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
#               | |
#               |_|
#
# Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
# See: https://www.mrpt.org/Authors - All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#*/

cmake_minimum_required(VERSION 3.16)
project(mrpt_examples_cpp)

find_package(mrpt_common REQUIRED)

# Process all subdirectories containing CMakeLists.txt files
file(GLOB SUBDIRS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "*")

foreach(SUBDIR ${SUBDIRS})
  set(SUBDIR_PATH "${CMAKE_CURRENT_SOURCE_DIR}/${SUBDIR}")
  
  # Check if subdirectory exists and contains CMakeLists.txt
  if(IS_DIRECTORY "${SUBDIR_PATH}" AND EXISTS "${SUBDIR_PATH}/CMakeLists.txt")
    add_subdirectory(${SUBDIR})
  endif()
endforeach()