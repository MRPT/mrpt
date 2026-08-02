/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

#include <gtest/gtest.h>
#include <mrpt/poses/CPointPDFGaussian.h>
#include <mrpt/poses/CPointPDFParticles.h>
#include <mrpt/poses/CPointPDFSOG.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/poses/CPose3DPDFParticles.h>
#include <mrpt/poses/CPose3DPDFSOG.h>
#include <mrpt/poses/CPose3DQuatPDFGaussian.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/poses/CPosePDFParticles.h>
#include <mrpt/poses/CPosePDFSOG.h>
#include <mrpt/viz/pose_pdfs.h>

using namespace mrpt::viz;
using namespace mrpt::poses;

TEST(pose_pdfs, PosePDFGaussian)
{
  CPosePDFGaussian pdf(mrpt::poses::CPose2D(1, 2, 0.3));
  auto obj = posePDF2opengl(pdf);
  ASSERT_TRUE(obj);
  EXPECT_GT(obj->size(), 0u);
}

TEST(pose_pdfs, PosePDFSOG)
{
  CPosePDFSOG pdf(2);
  auto obj = posePDF2opengl(pdf);
  ASSERT_TRUE(obj);
}

TEST(pose_pdfs, PosePDFParticles)
{
  CPosePDFParticles pdf;
  pdf.resetDeterministic(mrpt::math::TPose2D(0, 0, 0), 5);
  auto obj = posePDF2opengl(pdf);
  ASSERT_TRUE(obj);
  EXPECT_GT(obj->size(), 0u);
}

TEST(pose_pdfs, PointPDFGaussian)
{
  CPointPDFGaussian pdf(mrpt::poses::CPoint3D(1, 2, 3));
  auto obj = posePDF2opengl(pdf);
  ASSERT_TRUE(obj);
}

TEST(pose_pdfs, PointPDFSOG)
{
  CPointPDFSOG pdf(2);
  auto obj = posePDF2opengl(pdf);
  ASSERT_TRUE(obj);
}

TEST(pose_pdfs, PointPDFParticles)
{
  CPointPDFParticles pdf;
  pdf.setSize(5, mrpt::math::TPoint3Df(0, 0, 0));
  auto obj = posePDF2opengl(pdf);
  ASSERT_TRUE(obj);
}

TEST(pose_pdfs, Pose3DPDFGaussian)
{
  CPose3DPDFGaussian pdf(mrpt::poses::CPose3D(1, 2, 3, 0.1, 0.2, 0.3));
  auto obj = posePDF2opengl(pdf);
  ASSERT_TRUE(obj);
}

TEST(pose_pdfs, Pose3DPDFSOG)
{
  CPose3DPDFSOG pdf(2);
  auto obj = posePDF2opengl(pdf);
  ASSERT_TRUE(obj);
}

TEST(pose_pdfs, Pose3DPDFParticles)
{
  CPose3DPDFParticles pdf;
  pdf.resetDeterministic(mrpt::math::TPose3D(0, 0, 0, 0, 0, 0), 5);
  auto obj = posePDF2opengl(pdf);
  ASSERT_TRUE(obj);
}

TEST(pose_pdfs, Pose3DQuatPDFGaussian)
{
  CPose3DQuatPDFGaussian pdf(
      mrpt::poses::CPose3DQuat(mrpt::poses::CPose3D(1, 2, 3, 0.1, 0.2, 0.3)));
  auto obj = posePDF2opengl(pdf);
  ASSERT_TRUE(obj);
}
