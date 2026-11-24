# ------------------------------------------------------------------------------
#        Mobile Robot Programming Toolkit (MRPT)
#
# Copyright (c) 2005-2026, Jose Luis Blanco-Claraco, contributors (see Git history)
# All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
# ------------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.16)

# Tell CMake we'll use C++ for use in its tests/flags
project(mrpt_img LANGUAGES C CXX)

# MRPT CMake scripts: "mrpt_xxx()"
find_package(mrpt_common REQUIRED)
find_package(mrpt_io REQUIRED)
find_package(mrpt_config REQUIRED)
find_package(mrpt_math REQUIRED)
find_package(Eigen3 REQUIRED)

# Include Asian fonts in mrpt::img::CCanvas ?
set(MRPT_HAS_ASIAN_FONTS ON CACHE BOOL "Enable Asian fonts in mrpt::img::CCanvas (increases library size).")

# Optional: decode JPEG with libjpeg-turbo (TurboJPEG API), faster than the
# embedded stb_image, which remains the fallback and decodes all other formats.
set(MRPT_WITH_LIBJPEG_TURBO ON CACHE BOOL "Decode JPEG images with libjpeg-turbo, if found.")
set(CMAKE_MRPT_HAS_LIBJPEG_TURBO 0)
if (MRPT_WITH_LIBJPEG_TURBO)
  find_package(PkgConfig QUIET)
  if (PKG_CONFIG_FOUND)
    pkg_check_modules(PC_TURBOJPEG QUIET libturbojpeg)
  endif()
  find_path(TURBOJPEG_INCLUDE_DIR
    NAMES turbojpeg.h
    HINTS ${PC_TURBOJPEG_INCLUDE_DIRS} ${libjpeg-turbo_ROOT_DIR}/include)
  find_library(TURBOJPEG_LIBRARY
    NAMES turbojpeg
    HINTS ${PC_TURBOJPEG_LIBRARY_DIRS} ${libjpeg-turbo_ROOT_DIR}/lib)
  if (TURBOJPEG_INCLUDE_DIR AND TURBOJPEG_LIBRARY)
    # TJFLAG_STOPONWARNING requires libjpeg-turbo >= 2.0:
    include(CheckSymbolExists)
    set(CMAKE_REQUIRED_INCLUDES ${TURBOJPEG_INCLUDE_DIR})
    check_symbol_exists(TJFLAG_STOPONWARNING "turbojpeg.h" MRPT_TURBOJPEG_HAS_STOPONWARNING)
    unset(CMAKE_REQUIRED_INCLUDES)
    if (MRPT_TURBOJPEG_HAS_STOPONWARNING)
      set(CMAKE_MRPT_HAS_LIBJPEG_TURBO 1)
    endif()
  endif()
  if (CMAKE_MRPT_HAS_LIBJPEG_TURBO)
    message(STATUS "mrpt_img: JPEG decoding with libjpeg-turbo: ${TURBOJPEG_LIBRARY}")
  else()
    message(STATUS "mrpt_img: libjpeg-turbo >= 2.0 not found, JPEG decoding with stb_image")
  endif()
endif()

# define lib:
set(LIB_SOURCES
  src/camera_geometry.cpp
  src/CCanvas.cpp
  src/CImage_bayer.cpp
  src/CImage_impl.cpp
  src/CImage_impl.h
  src/CImage_jpeg.cpp
  src/CImage_jpeg.h
  src/CImage_loadXPM.cpp
  src/CImage.cpp
  src/CImage.SSE2.cpp
  src/CImage.SSEx.h
  src/CImage.SSSE3.cpp
  src/CImagePyramid.cpp
  src/CMappedImage.cpp
  src/color_maps.cpp
  src/CStereoRectifyMap.cpp
  src/CUndistortMap.cpp
  src/CVideoFileWriter.cpp
  src/registerAllClasses.cpp
  src/TCamera.cpp
  src/TColor.cpp
  src/TPixelCoord.cpp
  src/TStereoCamera.cpp
)

set(LIB_UNIT_TEST_SOURCES
  tests/camera_geometry_unittest.cpp
  tests/CCanvas_unittest.cpp
  tests/CImage_jpeg_unittest.cpp
  tests/CImage_legacy_serialization_unittest.cpp
  tests/CImage_loadFromBayerBuffer_unittest.cpp
  tests/CImage_loadFromEncodedBuffer_unittest.cpp
  tests/CImage_loadXPM_unittest.cpp
  tests/CImage_SSEx_unittest.cpp
  tests/CImage_unittest.cpp
  tests/CImagePyramid_unittest.cpp
  tests/CMappedImage_unittest.cpp
  tests/color_maps_unittest.cpp
  tests/CUndistortMap_unittest.cpp
  tests/CVideoFileWriter_unittest.cpp
  tests/TCamera_unittest.cpp
  tests/TColor_unittest.cpp
  tests/TPixelCoord_unittest.cpp
  tests/TStereoCamera_unittest.cpp
)

set(LIB_PUBLIC_HEADERS
  include/mrpt/img/camera_geometry.h
  include/mrpt/img/CCanvas.h
  include/mrpt/img/CImage.h
  include/mrpt/img/CImage_Eigen.h
  include/mrpt/img/CImagePyramid.h
  include/mrpt/img/CMappedImage.h
  include/mrpt/img/color_maps.h
  include/mrpt/img/CStereoRectifyMap.h
  include/mrpt/img/CUndistortMap.h
  include/mrpt/img/CVideoFileWriter.h
  include/mrpt/img/DistortionModel.h
  include/mrpt/img/registerAllClasses.h
  include/mrpt/img/TCamera.h
  include/mrpt/img/TColor.h
  include/mrpt/img/TPixelCoord.h
  include/mrpt/img/TStereoCamera.h
)

mrpt_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES ${LIB_SOURCES} ${LIB_PUBLIC_HEADERS}
  UNIT_TEST_SOURCES ${LIB_UNIT_TEST_SOURCES}
  PUBLIC_LINK_LIBRARIES
    mrpt::mrpt_config
    mrpt::mrpt_io
    mrpt::mrpt_math
  PRIVATE_LINK_LIBRARIES
    Eigen3::Eigen
  CMAKE_DEPENDENCIES
    mrpt_config
    mrpt_io
    mrpt_math
  # Other imported targets:
)

if (CMAKE_MRPT_HAS_LIBJPEG_TURBO)
  target_include_directories(${PROJECT_NAME} PRIVATE ${TURBOJPEG_INCLUDE_DIR})
  target_link_libraries(${PROJECT_NAME} PRIVATE ${TURBOJPEG_LIBRARY})
endif()

# Python wrapper:
mrpt_add_python_module(img
  python_bindings/${PROJECT_NAME}_py.cpp
)

mrpt_add_python_binding_test(img NEEDS_NUMPY
  SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/tests/python_img_test.py
)

