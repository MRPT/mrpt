# ------------------------------------------------------------------------------
#        Mobile Robot Programming Toolkit (MRPT)
#
# Copyright (c) 2005-2026, Jose Luis Blanco-Claraco, contributors (see Git history)
# All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
# ------------------------------------------------------------------------------
#
# Standalone helper to generate undistortion ground truth using OpenCV.
# Not part of the main MRPT build. Build separately:
#
#   mkdir build && cd build
#   cmake .. && cmake --build .
#   ./generate_undistort_groundtruth <input.png> <output.png>
#
cmake_minimum_required(VERSION 3.16)
project(generate_undistort_groundtruth LANGUAGES CXX)

find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs calib3d)

add_executable(generate_undistort_groundtruth
  generate_undistort_groundtruth.cpp
)
target_link_libraries(generate_undistort_groundtruth PRIVATE
  ${OpenCV_LIBS}
)
target_compile_features(generate_undistort_groundtruth PRIVATE cxx_std_17)
