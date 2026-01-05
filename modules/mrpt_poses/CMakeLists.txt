# ------------------------------------------------------------------------------
#        Mobile Robot Programming Toolkit (MRPT)
#
# Copyright (c) 2005-2025, Jose Luis Blanco-Claraco, contributors (see Git history)
# All rights reserved.
# Released under BSD-3 license. See LICENSE file
# ------------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.16)

# Tell CMake we'll use C++ for use in its tests/flags
project(mrpt_poses LANGUAGES C CXX)

# MRPT CMake scripts: "mrpt_xxx()"
find_package(mrpt_common REQUIRED)
find_package(mrpt_bayes REQUIRED)

# define lib:
set(LIB_SRCS
  src/CPoint2D.cpp
  src/CPoint2DPDF.cpp
  src/CPoint2DPDFGaussian.cpp
  src/CPoint2DPDF_unittest.cpp
  src/CPoint3D.cpp
  src/CPoint.cpp
  src/CPointPDF.cpp
  src/CPointPDFGaussian.cpp
  src/CPointPDFParticles.cpp
  src/CPointPDFSOG.cpp
  src/CPoint_unittest.cpp
  src/CPose2D.cpp
  src/CPose2DInterpolator.cpp
  src/CPose2DInterpolator_unittest.cpp
  src/CPose3D.cpp
  src/CPose3DInterpolator.cpp
  src/CPose3DInterpolator_unittest.cpp
  src/CPose3DPDF.cpp
  src/CPose3DPDFGaussian.cpp
  src/CPose3DPDFGaussianInf.cpp
  src/CPose3DPDFGaussian_unittest.cpp
  src/CPose3DPDFGrid.cpp
  src/CPose3DPDFGrid_unittest.cpp
  src/CPose3DPDFParticles.cpp
  src/CPose3DPDFSOG.cpp
  src/CPose3DQuat.cpp
  src/CPose3DQuatPDF.cpp
  src/CPose3DQuatPDFGaussian.cpp
  src/CPose3DQuatPDFGaussianInf.cpp
  src/CPose3DQuatPDFGaussian_unittest.cpp
  src/CPose3DQuat_unittest.cpp
  src/CPose3D_unittest.cpp
  src/CPoseInterpolatorBase.hpp
  src/CPosePDF.cpp
  src/CPosePDFGaussian.cpp
  src/CPosePDFGaussianInf.cpp
  src/CPosePDFGaussian_unittest.cpp
  src/CPosePDFGrid.cpp
  src/CPosePDFGrid_unittest.cpp
  src/CPosePDFParticles.cpp
  src/CPosePDFSOG.cpp
  src/CPoseRandomSampler.cpp
  src/CPoseRandomSampler_unittest.cpp
  src/CPoses2DSequence.cpp
  src/CPoses3DSequence.cpp
  src/CQuaternion_unittest.cpp
  src/CRobot2DPoseEstimator.cpp
  src/CRobot2DPoseEstimator_unittest.cpp
  src/CSchemeArchive_unittest.cpp
  src/FrameTransformer.cpp
  src/FrameTransformer_unittest.cpp
  src/Lie/LieTraits_unittest.cpp
  src/Lie/SE.cpp
  src/Lie/SO.cpp
  src/matrix_serialization_unittest.cpp
  src/registerAllClasses.cpp
  src/sensor_poses.cpp
  src/sensor_poses_unittest.cpp
  src/SO_SE_average.cpp
  src/SO_SE_average_unittest.cpp
)

set(LIB_PUBLIC_HDRS
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

mrpt_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES ${LIB_SRCS} ${LIB_PUBLIC_HDRS}
  PUBLIC_LINK_LIBRARIES
    mrpt::mrpt_bayes
#  PRIVATE_LINK_LIBRARIES
  CMAKE_DEPENDENCIES
    mrpt_bayes
  # Other imported targets:
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

