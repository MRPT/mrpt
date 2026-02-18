# ------------------------------------------------------------------------------
#        Mobile Robot Programming Toolkit (MRPT)
#
# Copyright (c) 2005-2026, Jose Luis Blanco-Claraco, contributors (see Git history)
# All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
# ------------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.16)
project(mrpt_vision LANGUAGES C CXX)

find_package(mrpt_common REQUIRED)
find_package(mrpt_obs REQUIRED)

# define lib:
set(LIB_SOURCES
  src/CDifodo.cpp
  src/CImagePyramid.cpp
  src/CStereoRectifyMap.cpp
  src/CUndistortMap.cpp
  src/CVideoFileWriter.cpp
  src/checkerboard_cam_calib.cpp
  src/checkerboard_find_corners.cpp
  src/chessboard_stereo_camera_calib.cpp
  src/chessboard_stereo_camera_calib_internal.h
  src/registerAllClasses.cpp
  src/tracking.cpp
  src/tracking_KL.cpp
  src/types.cpp
  src/vision_utils.cpp
  tests/chessboard_stereo_camera_calib_unittest.cpp
)

set(LIB_PUBLIC_HEADERS
  include/mrpt/vision/CDifodo.h
  include/mrpt/vision/chessboard_camera_calib.h
  include/mrpt/vision/chessboard_find_corners.h
  include/mrpt/vision/chessboard_stereo_camera_calib.h
  include/mrpt/vision/CImagePyramid.h
  include/mrpt/vision/CStereoRectifyMap.h
  include/mrpt/vision/CUndistortMap.h
  include/mrpt/vision/CVideoFileWriter.h
  include/mrpt/vision/pnp_algos.h
  include/mrpt/vision/registerAllClasses.h
  include/mrpt/vision/TKeyPoint.h
  include/mrpt/vision/tracking.h
  include/mrpt/vision/types.h
  include/mrpt/vision/utils.h
)

mrpt_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES ${LIB_SOURCES} ${LIB_PUBLIC_HEADERS}
  PUBLIC_LINK_LIBRARIES
    mrpt::mrpt_obs
  CMAKE_DEPENDENCIES
    mrpt_obs
)
