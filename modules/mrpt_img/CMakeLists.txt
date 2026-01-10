# ------------------------------------------------------------------------------
#        Mobile Robot Programming Toolkit (MRPT)
#
# Copyright (c) 2005-2026, Jose Luis Blanco-Claraco, contributors (see Git history)
# All rights reserved.
# Released under BSD-3 license. See LICENSE file
# ------------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.16)

# Tell CMake we'll use C++ for use in its tests/flags
project(mrpt_img LANGUAGES C CXX)

# MRPT CMake scripts: "mrpt_xxx()"
find_package(mrpt_common REQUIRED)
find_package(mrpt_io REQUIRED)
find_package(mrpt_config REQUIRED)
find_package(mrpt_math REQUIRED)

# Include Asian fonts in mrpt::img::CCanvas ?
set(MRPT_HAS_ASIAN_FONTS ON CACHE BOOL "Enable Asian fonts in mrpt::img::CCanvas (increases library size).")

# define lib:
set(LIB_SRCS
  src/CCanvas.cpp
  src/CImage.cpp
  src/CImage_impl.cpp
  src/CImage_impl.h
  src/CImage_loadXPM.cpp
  src/CImage.SSE2.cpp
  src/CImage.SSEx.h
  src/CImage.SSSE3.cpp
  src/CMappedImage.cpp
  src/color_maps.cpp
  src/registerAllClasses.cpp
  src/TCamera.cpp
  src/TColor.cpp
  src/TColorManager.cpp
  src/TPixelCoord.cpp
  src/TStereoCamera.cpp
  tests/CImage_unittest.cpp
  tests/color_maps_unittest.cpp
  tests/TCamera_unittest.cpp
)

set(LIB_PUBLIC_HDRS
  include/mrpt/img/CCanvas.h
  include/mrpt/img/CImage.h
  include/mrpt/img/CMappedImage.h
  include/mrpt/img/color_maps.h
  include/mrpt/img/DistortionModel.h
  include/mrpt/img/registerAllClasses.h
  include/mrpt/img/TCamera.h
  include/mrpt/img/TColor.h
  include/mrpt/img/TColorManager.h
  include/mrpt/img/TPixelCoord.h
  include/mrpt/img/TStereoCamera.h
)

mrpt_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES ${LIB_SRCS} ${LIB_PUBLIC_HDRS}
  PUBLIC_LINK_LIBRARIES
    mrpt::mrpt_config
    mrpt::mrpt_io
    mrpt::mrpt_math
  CMAKE_DEPENDENCIES
    mrpt_config
    mrpt_io
    mrpt_math
  # Other imported targets:
)

# Python wrapper:
mrpt_add_python_module(img
  python_bindings/${PROJECT_NAME}_py.cpp
)

