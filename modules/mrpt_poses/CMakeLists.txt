# ------------------------------------------------------------------------------
#        Mobile Robot Programming Toolkit (MRPT)
#
# Copyright (c) 2005-2026, Jose Luis Blanco-Claraco, contributors (see Git history)
# All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
# ------------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.16)

# Tell CMake we'll use C++ for use in its tests/flags
project(mrpt_poses LANGUAGES C CXX)

# MRPT CMake scripts: "mrpt_xxx()"
find_package(mrpt_common REQUIRED)
find_package(mrpt_bayes REQUIRED)

# define lib:
set(LIB_SOURCES
  src/CPointPDFSOG.cpp
  src/CPosePDFParticles.cpp
  src/CPose2DInterpolator.cpp
  src/sensor_poses.cpp
  src/CPose3DPDFSOG.cpp
  src/CPose3DPDFGaussian.cpp
  src/CPose3DQuat.cpp
  src/CPoint2D.cpp
  src/FrameTransformer.cpp
  src/CPose3DPDFParticles.cpp
  src/CPoseRandomSampler.cpp
  src/CPointPDFParticles.cpp
  src/CRobot2DPoseEstimator.cpp
  src/CPoseInterpolatorBase.hpp
  src/SO_SE_average.cpp
  src/CPointPDFGaussian.cpp
  src/CPosePDFGaussian.cpp
  src/CPosePDF.cpp
  src/CPoint.cpp
  src/CPose2D.cpp
  src/CPose3D.cpp
  src/CPose3DPDFGaussianInf.cpp
  src/CPoint2DPDF.cpp
  src/CPose3DPDFGrid.cpp
  src/CPose3DPDF.cpp
  src/CPointPDF.cpp
  src/CPose3DQuatPDF.cpp
  src/registerAllClasses.cpp
  src/CPosePDFGaussianInf.cpp
  src/CPose3DQuatPDFGaussianInf.cpp
  src/CPoint3D.cpp
  src/CPose3DInterpolator.cpp
  src/CPoses2DSequence.cpp
  src/CPose3DQuatPDFGaussian.cpp
  src/Lie/SE.cpp
  src/Lie/SO.cpp
  src/CPoint2DPDFGaussian.cpp
  src/CPoses3DSequence.cpp
  src/CPosePDFSOG.cpp
  src/CPosePDFGrid.cpp
  # tests
  tests/CPosePDFGrid_unittest.cpp
  tests/CQuaternion_unittest.cpp
  tests/CPose3D_unittest.cpp
  tests/CPoseRandomSampler_unittest.cpp
  tests/CRobot2DPoseEstimator_unittest.cpp
  tests/FrameTransformer_unittest.cpp
  tests/CPoint_unittest.cpp
  tests/CPose3DPDFGrid_unittest.cpp
  tests/CPose2DInterpolator_unittest.cpp
  tests/SO_SE_average_unittest.cpp
  tests/CPoint2DPDF_unittest.cpp
  tests/CPose3DQuat_unittest.cpp
  tests/CPose3DInterpolator_unittest.cpp
  tests/CPosePDFGaussian_unittest.cpp
  tests/CSchemeArchive_unittest.cpp
  tests/CPose3DQuatPDFGaussian_unittest.cpp
  tests/sensor_poses_unittest.cpp
  tests/CPose3DPDFGaussian_unittest.cpp
  tests/matrix_serialization_unittest.cpp
  tests/Lie_unittest.cpp
)

set(LIB_PUBLIC_HEADERS
  include/mrpt/poses/poses_frwds.h
  include/mrpt/poses/CPoint2DPDF.h
  include/mrpt/poses/CPose2D.h
  include/mrpt/poses/SO_SE_average.h
  include/mrpt/poses/CPose2DGridTemplate.h
  include/mrpt/poses/CPoseRandomSampler.h
  include/mrpt/poses/CPose3DPDFGaussian.h
  include/mrpt/poses/sensor_poses.h
  include/mrpt/poses/CPose3DPDFGaussianInf.h
  include/mrpt/poses/CPose2DInterpolator.h
  include/mrpt/poses/CPose3DInterpolator.h
  include/mrpt/poses/CPose3DPDFParticles.h
  include/mrpt/poses/registerAllClasses.h
  include/mrpt/poses/CPose.h
  include/mrpt/poses/CPosePDFParticles.h
  include/mrpt/poses/CPoint2DPDFGaussian.h
  include/mrpt/poses/CPosePDFGaussianInf.h
  include/mrpt/poses/CPose3DGridTemplate.h
  include/mrpt/poses/CPosePDFGrid.h
  include/mrpt/poses/CRobot2DPoseEstimator.h
  include/mrpt/poses/CPoseOrPoint.h
  include/mrpt/poses/CPoint3D.h
  include/mrpt/poses/CPose3DQuat.h
  include/mrpt/poses/CPose3D.h
  include/mrpt/poses/CPoseOrPoint_detail.h
  include/mrpt/poses/CPosePDFGaussian.h
  include/mrpt/poses/CPointPDF.h
  include/mrpt/poses/CPose3DPDFSOG.h
  include/mrpt/poses/CPointPDFParticles.h
  include/mrpt/poses/CPose3DQuatPDFGaussian.h
  include/mrpt/poses/CPosePDFSOG.h
  include/mrpt/poses/FrameTransformer.h
  include/mrpt/poses/CPoint.h
  include/mrpt/poses/CPose3DPDFGrid.h
  include/mrpt/poses/CPosePDF.h
  include/mrpt/poses/CPoint2D.h
  include/mrpt/poses/CPoses2DSequence.h
  include/mrpt/poses/CPoses3DSequence.h
  include/mrpt/poses/pose_utils.h
  include/mrpt/poses/CPointPDFGaussian.h
  include/mrpt/poses/CPose3DPDF.h
  include/mrpt/poses/CPointPDFSOG.h
  include/mrpt/poses/CPose3DQuatPDFGaussianInf.h
  include/mrpt/poses/gtsam_wrappers.h
  include/mrpt/poses/CPoseInterpolatorBase.h
  include/mrpt/poses/Lie/Euclidean.h
  include/mrpt/poses/Lie/SO.h
  include/mrpt/poses/Lie/SE.h
  include/mrpt/poses/CPose3DQuatPDF.h
)

if (BUILD_TESTING)
  find_package(mrpt_io REQUIRED) # for unit tests
endif()

mrpt_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES ${LIB_SOURCES} ${LIB_PUBLIC_HEADERS}
  PUBLIC_LINK_LIBRARIES
    mrpt::mrpt_bayes
#  PRIVATE_LINK_LIBRARIES
  CMAKE_DEPENDENCIES
    mrpt_bayes
  UNITTEST_LINK_LIBRARIES
    mrpt::mrpt_io
)

target_precompile_headers(${PROJECT_NAME} PRIVATE
    <vector>
    <string>
    <memory>
    <mrpt/poses/CPose3D.h>
)

# Python wrapper:
mrpt_add_python_module(poses
  python_bindings/${PROJECT_NAME}_py.cpp
)

