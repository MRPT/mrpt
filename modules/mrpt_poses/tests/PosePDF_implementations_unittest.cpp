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
#include <mrpt/math/wrap2pi.h>
#include <mrpt/poses/CPoint2DPDFGaussian.h>
#include <mrpt/poses/CPointPDFGaussian.h>
#include <mrpt/poses/CPointPDFParticles.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/poses/CPose3DPDFGaussianInf.h>
#include <mrpt/poses/CPose3DPDFParticles.h>
#include <mrpt/poses/CPose3DPDFSOG.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/poses/CPosePDFGrid.h>
#include <mrpt/poses/CPosePDFSOG.h>
#include <mrpt/random.h>

using namespace mrpt;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace mrpt::random;

// --- CPose3DPDFGaussian ---

TEST(CPose3DPDFGaussian, bayesianFusion)
{
  CMatrixDouble66 cov1;
  cov1.setIdentity();
  cov1 *= 0.1;
  CPose3DPDFGaussian p1(CPose3D(1, 0, 0, 0, 0, 0), cov1);

  CMatrixDouble66 cov2;
  cov2.setIdentity();
  cov2 *= 0.1;
  CPose3DPDFGaussian p2(CPose3D(2, 0, 0, 0, 0, 0), cov2);

  CPose3DPDFGaussian result;
  result.bayesianFusion(p1, p2);

  // Mean should be between p1 and p2
  EXPECT_NEAR(result.mean.x(), 1.5, 0.01);
  EXPECT_NEAR(result.mean.y(), 0.0, 0.01);
  // Covariance should be smaller than either input
  EXPECT_LT(result.cov(0, 0), cov1(0, 0));
}

TEST(CPose3DPDFGaussian, evaluatePDF)
{
  CMatrixDouble66 cov;
  cov.setIdentity();
  cov *= 0.5;
  CPose3DPDFGaussian pdf(CPose3D(1, 2, 3, 0.1, 0.2, 0.3), cov);

  // PDF at mean should be the maximum
  double pAtMean = pdf.evaluatePDF(pdf.mean);
  double pAway = pdf.evaluatePDF(CPose3D(10, 10, 10, 0, 0, 0));
  EXPECT_GT(pAtMean, 0.0);
  EXPECT_GT(pAtMean, pAway);
}

TEST(CPose3DPDFGaussian, evaluateNormalizedPDF)
{
  CMatrixDouble66 cov;
  cov.setIdentity();
  CPose3DPDFGaussian pdf(CPose3D(0, 0, 0, 0, 0, 0), cov);

  // At mean, normalized PDF should be 1.0
  EXPECT_NEAR(pdf.evaluateNormalizedPDF(pdf.mean), 1.0, 1e-6);

  // Away from mean, should be less than 1
  EXPECT_LT(pdf.evaluateNormalizedPDF(CPose3D(5, 5, 5, 0, 0, 0)), 1.0);
}

// --- CPose3DPDFGaussianInf ---

TEST(CPose3DPDFGaussianInf, bayesianFusion)
{
  CMatrixDouble66 cov1;
  cov1.setIdentity();
  cov1 *= 0.1;
  CPose3DPDFGaussian g1(CPose3D(1, 0, 0, 0, 0, 0), cov1);
  CPose3DPDFGaussian g2(CPose3D(2, 0, 0, 0, 0, 0), cov1);

  CPose3DPDFGaussianInf p1, p2;
  p1.copyFrom(g1);
  p2.copyFrom(g2);

  CPose3DPDFGaussianInf result;
  result.bayesianFusion(p1, p2);

  EXPECT_NEAR(result.mean.x(), 1.5, 0.01);
}

TEST(CPose3DPDFGaussianInf, evaluatePDF)
{
  CMatrixDouble66 cov;
  cov.setIdentity();
  CPose3DPDFGaussian g(CPose3D(0, 0, 0, 0, 0, 0), cov);
  CPose3DPDFGaussianInf pdf;
  pdf.copyFrom(g);

  double pAtMean = pdf.evaluatePDF(pdf.mean);
  EXPECT_GT(pAtMean, 0.0);
  EXPECT_NEAR(pdf.evaluateNormalizedPDF(pdf.mean), 1.0, 1e-4);
}

// --- CPointPDFGaussian ---

TEST(CPointPDFGaussian, bayesianFusion)
{
  CPointPDFGaussian p1, p2;
  p1.mean = CPoint3D(1, 0, 0);
  p1.cov.setIdentity();
  p1.cov *= 0.1;

  p2.mean = CPoint3D(2, 0, 0);
  p2.cov.setIdentity();
  p2.cov *= 0.1;

  CPointPDFGaussian result;
  result.bayesianFusion(p1, p2);

  EXPECT_NEAR(result.mean.x(), 1.5, 0.01);
}

// --- CPoint2DPDFGaussian ---

TEST(CPoint2DPDFGaussian, bayesianFusion)
{
  CPoint2DPDFGaussian p1, p2;
  p1.mean = CPoint2D(1, 0);
  p1.cov.setIdentity();
  p1.cov *= 0.1;

  p2.mean = CPoint2D(2, 0);
  p2.cov.setIdentity();
  p2.cov *= 0.1;

  CPoint2DPDFGaussian result;
  result.bayesianFusion(p1, p2);

  EXPECT_NEAR(result.mean.x(), 1.5, 0.01);
}

// --- CPose3DPDFSOG ---

TEST(CPose3DPDFSOG, drawSingleSample)
{
  CPose3DPDFSOG sog(2);

  CMatrixDouble66 cov;
  cov.setIdentity();
  cov *= 0.01;

  auto it = sog.begin();
  it->val.mean = CPose3D(1, 0, 0, 0, 0, 0);
  it->val.cov = cov;
  it->log_w = 0.0;
  ++it;
  it->val.mean = CPose3D(2, 0, 0, 0, 0, 0);
  it->val.cov = cov;
  it->log_w = 0.0;

  CPose3D sample;
  sog.drawSingleSample(sample);

  // Sample should be near one of the modes
  bool nearMode1 = std::abs(sample.x() - 1.0) < 1.0;
  bool nearMode2 = std::abs(sample.x() - 2.0) < 1.0;
  EXPECT_TRUE(nearMode1 || nearMode2);
}

TEST(CPose3DPDFSOG, bayesianFusion)
{
  CPose3DPDFSOG s1(1), s2(1);

  CMatrixDouble66 cov;
  cov.setIdentity();
  cov *= 0.1;

  s1.begin()->val.mean = CPose3D(1, 0, 0, 0, 0, 0);
  s1.begin()->val.cov = cov;
  s1.begin()->log_w = 0.0;

  s2.begin()->val.mean = CPose3D(2, 0, 0, 0, 0, 0);
  s2.begin()->val.cov = cov;
  s2.begin()->log_w = 0.0;

  CPose3DPDFSOG result;
  result.bayesianFusion(s1, s2);

  EXPECT_EQ(result.size(), 1u);
}

// --- CPosePDFSOG ---

TEST(CPosePDFSOG, drawSingleSample)
{
  CPosePDFSOG sog(2);

  CMatrixDouble33 cov;
  cov.setIdentity();
  cov *= 0.01;

  sog[0].mean = CPose2D(1, 0, 0);
  sog[0].cov = cov;
  sog[0].log_w = 0.0;

  sog[1].mean = CPose2D(2, 0, 0);
  sog[1].cov = cov;
  sog[1].log_w = 0.0;

  CPose2D sample;
  sog.drawSingleSample(sample);

  bool nearMode1 = std::abs(sample.x() - 1.0) < 1.0;
  bool nearMode2 = std::abs(sample.x() - 2.0) < 1.0;
  EXPECT_TRUE(nearMode1 || nearMode2);
}

// --- CPose3DPDFParticles ---

TEST(CPose3DPDFParticles, drawSingleSample)
{
  CPose3DPDFParticles parts;
  parts.resetDeterministic(TPose3D(1, 2, 3, 0.1, 0.2, 0.3), 100);

  CPose3D sample;
  parts.drawSingleSample(sample);

  // All particles are at the same pose, so sample should match
  EXPECT_NEAR(sample.x(), 1.0, 1e-6);
  EXPECT_NEAR(sample.y(), 2.0, 1e-6);
  EXPECT_NEAR(sample.z(), 3.0, 1e-6);
}

TEST(CPose3DPDFParticles, operatorPlusEquals)
{
  CPose3DPDFParticles parts;
  parts.resetDeterministic(TPose3D(1, 0, 0, 0, 0, 0), 10);

  parts += CPose3D(1, 0, 0, 0, 0, 0);

  CPose3D mean;
  parts.getMean(mean);
  EXPECT_NEAR(mean.x(), 2.0, 1e-4);
}

// --- CPosePDFGrid ---

TEST(CPosePDFGrid, drawSingleSample)
{
  CPosePDFGrid grid(-2, 2, -2, 2, 0.5, DEG2RAD(10));

  // The constructor fills the grid uniformly; zero it out first, then
  // set a single cell to high probability:
  grid.fill(0.0);
  *grid.getByPos(0.0, 0.0, 0.0) = 1.0;
  grid.normalize();

  CPose2D sample;
  grid.drawSingleSample(sample);

  // Grid cell centers may not be exactly at 0 — allow resolution tolerance
  EXPECT_NEAR(sample.x(), 0.0, 1.0);
  EXPECT_NEAR(sample.y(), 0.0, 1.0);
}

// --- CPointPDFParticles ---

TEST(CPointPDFParticles, drawSingleSample)
{
  CPointPDFParticles parts;
  parts.setSize(50, TPoint3Df(1, 2, 3));

  CPoint3D sample;
  parts.drawSingleSample(sample);

  EXPECT_NEAR(sample.x(), 1.0, 1e-4);
  EXPECT_NEAR(sample.y(), 2.0, 1e-4);
  EXPECT_NEAR(sample.z(), 3.0, 1e-4);
}
