# ------------------------------------------------------------------------------
#        Mobile Robot Programming Toolkit (MRPT)
#
# Copyright (c) 2005-2026, Jose Luis Blanco-Claraco, contributors (see Git history)
# All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
# ------------------------------------------------------------------------------

# Minimal downstream project, built against the C++ SDK inside the MRPT wheel
# (CMAKE_PREFIX_PATH = mrpt.sdk.get_cmake_prefix_path()). Run by test_wheel.py.

cmake_minimum_required(VERSION 3.21)
project(mrpt_sdk_consumer LANGUAGES CXX)

find_package(mrpt_poses REQUIRED)
find_package(mrpt_maps REQUIRED)
find_package(mrpt_slam REQUIRED)  # it also finds the SDK oneTBB
find_package(mrpt_gui REQUIRED)   # it also finds the SDK Eigen and nanogui (GLFW)

add_executable(sdk_consumer main.cpp)
target_link_libraries(sdk_consumer PRIVATE
  mrpt::mrpt_poses mrpt::mrpt_maps mrpt::mrpt_slam mrpt::mrpt_gui)
