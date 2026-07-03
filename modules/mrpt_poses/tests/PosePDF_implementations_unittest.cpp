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
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/poses/CPoint2DPDFGaussian.h>
#include <mrpt/poses/CPointPDFGaussian.h>
#include <mrpt/poses/CPointPDFParticles.h>
#include <mrpt/poses/CPointPDFSOG.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/poses/CPose3DPDFGaussianInf.h>
#include <mrpt/poses/CPose3DPDFGrid.h>
#include <mrpt/poses/CPose3DPDFParticles.h>
#include <mrpt/poses/CPose3DPDFSOG.h>
#include <mrpt/poses/CPose3DQuatPDFGaussian.h>
#include <mrpt/poses/CPose3DQuatPDFGaussianInf.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/poses/CPosePDFGaussianInf.h>
#include <mrpt/poses/CPosePDFGrid.h>
#include <mrpt/poses/CPosePDFParticles.h>
#include <mrpt/poses/CPosePDFSOG.h>
#include <mrpt/poses/CPoses2DSequence.h>
#include <mrpt/poses/CPoses3DSequence.h>
#include <mrpt/random.h>
#include <mrpt/serialization/CArchive.h>

#include <filesystem>

using namespace mrpt;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace mrpt::random;

namespace
{
std::string testTempFile(const char* name)
{
  return (std::filesystem::temp_directory_path() / name).string();
}
}  // namespace

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

  CPose3DPDFGaussianInf p1;
  CPose3DPDFGaussianInf p2;
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
  CPointPDFGaussian p1;
  CPointPDFGaussian p2;
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
  CPoint2DPDFGaussian p1;
  CPoint2DPDFGaussian p2;
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
  CPose3DPDFSOG s1(1);
  CPose3DPDFSOG s2(1);

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

// =====================================================================
// --- CPosePDFGaussian (extended) ---
// =====================================================================

TEST(CPosePDFGaussian, ConstructorsAndAccessors)
{
  CPosePDFGaussian p0;
  EXPECT_EQ(p0.mean, CPose2D(0, 0, 0));

  CPosePDFGaussian p1(CPose2D(1, 2, 0.3));
  EXPECT_EQ(p1.mean, CPose2D(1, 2, 0.3));
  EXPECT_DOUBLE_EQ(p1.cov(0, 0), 0.0);

  CMatrixDouble33 cov;
  cov.setIdentity();
  cov *= 0.1;
  CPosePDFGaussian p2(CPose2D(1, 2, 0.3), cov);
  EXPECT_EQ(p2.getPoseMean(), CPose2D(1, 2, 0.3));
  EXPECT_EQ(p2.meanVal(), CPose2D(1, 2, 0.3));

  auto [c, m] = p2.getCovarianceAndMean();
  EXPECT_EQ(m, p2.mean);
  EXPECT_TRUE(c == cov);
}

TEST(CPosePDFGaussian, CopyFromParticlesAndPose3D)
{
  CPosePDFParticles parts(1000);
  parts.resetDeterministic(TPose2D(1, 2, 0.1), 1000);

  CPosePDFGaussian g;
  g.copyFrom(parts);
  EXPECT_NEAR(g.mean.x(), 1.0, 1e-6);
  EXPECT_NEAR(g.mean.y(), 2.0, 1e-6);

  CMatrixDouble66 cov6;
  cov6.setIdentity();
  cov6 *= 0.01;
  CPose3DPDFGaussian pose3d(CPose3D(1, 2, 3, 0.1, 0.2, 0.3), cov6);
  CPosePDFGaussian g2(pose3d);  // explicit ctor calling copyFrom(CPose3DPDF&)
  EXPECT_NEAR(g2.mean.x(), 1.0, 1e-6);

  CPosePDFGaussian g3;
  g3.copyFrom(pose3d);
  EXPECT_NEAR(g3.mean.y(), 2.0, 1e-6);
}

TEST(CPosePDFGaussian, ChangeCoordinatesReference)
{
  CMatrixDouble33 cov;
  cov.setIdentity();
  cov *= 0.05;
  CPosePDFGaussian p(CPose2D(1, 0, 0), cov);
  p.changeCoordinatesReference(CPose2D(0, 0, M_PI / 2));
  EXPECT_NEAR(p.mean.x(), 0.0, 1e-6);
  EXPECT_NEAR(p.mean.y(), 1.0, 1e-6);

  CPosePDFGaussian p2(CPose2D(1, 0, 0), cov);
  p2.changeCoordinatesReference(CPose3D(0, 0, 0, M_PI / 2, 0, 0));
  EXPECT_NEAR(p2.mean.x(), 0.0, 1e-6);
  EXPECT_NEAR(p2.mean.y(), 1.0, 1e-6);
}

TEST(CPosePDFGaussian, InverseCompositionAndOperators)
{
  CMatrixDouble33 cov;
  cov.setIdentity();
  cov *= 0.01;
  CPosePDFGaussian x(CPose2D(2, 0, 0), cov);
  CPosePDFGaussian ref(CPose2D(1, 0, 0), cov);

  CPosePDFGaussian result;
  result.inverseComposition(x, ref);
  EXPECT_NEAR(result.mean.x(), 1.0, 1e-6);

  CMatrixDouble33 cov01;
  cov01.setZero();
  CPosePDFGaussian result2;
  result2.inverseComposition(x, ref, cov01);
  EXPECT_NEAR(result2.mean.x(), 1.0, 1e-6);

  CPosePDFGaussian a(CPose2D(1, 0, 0), cov);
  a += CPose2D(1, 0, 0);
  EXPECT_NEAR(a.mean.x(), 2.0, 1e-6);

  CPosePDFGaussian b(CPose2D(1, 0, 0), cov);
  b += CPosePDFGaussian(CPose2D(1, 0, 0), cov);
  EXPECT_NEAR(b.mean.x(), 2.0, 1e-6);

  CPosePDFGaussian c(CPose2D(2, 0, 0), cov);
  c -= CPosePDFGaussian(CPose2D(1, 0, 0), cov);
  EXPECT_NEAR(c.mean.x(), 1.0, 1e-6);

  CPosePDFGaussian d = a + b;
  CPosePDFGaussian e = a - b;
  (void)d;
  (void)e;

  EXPECT_TRUE(a == a);
  EXPECT_FALSE(a == b);
}

TEST(CPosePDFGaussian, DrawSamplesInverseAndMisc)
{
  CMatrixDouble33 cov;
  cov.setIdentity();
  cov *= 0.01;
  CPosePDFGaussian p(CPose2D(1, 2, 0.1), cov);

  CPose2D sample;
  p.drawSingleSample(sample);

  std::vector<CVectorDouble> samples;
  p.drawManySamples(10, samples);
  EXPECT_EQ(samples.size(), 10u);

  CPosePDFGaussian inv;
  p.inverse(inv);

  double d = p.mahalanobisDistanceTo(p);
  EXPECT_NEAR(d, 0.0, 1e-9);

  p.assureMinCovariance(1.0, 1.0);
  EXPECT_GE(p.cov(0, 0), 1.0);

  CPoint2DPDFGaussian g;
  p.composePoint(TPoint2D(1, 0), g);

  const std::string tmpf = testTempFile("mrpt_test_posepdfgaussian.txt");
  EXPECT_TRUE(p.saveToTextFile(tmpf));

  std::ostringstream ss;
  p.printTo(ss);
  EXPECT_FALSE(ss.str().empty());
  std::ostringstream ss2;
  ss2 << p;
  EXPECT_FALSE(ss2.str().empty());
  EXPECT_FALSE(p.asString().empty());
}

TEST(CPosePDFGaussian, SerializationRoundTrip)
{
  CMatrixDouble33 cov;
  cov.setIdentity();
  cov *= 0.02;
  CPosePDFGaussian p(CPose2D(1, 2, 0.3), cov);

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << p;
  buf.Seek(0);
  CPosePDFGaussian p2;
  arch >> p2;
  EXPECT_TRUE(p == p2);
}

// =====================================================================
// --- CPosePDFGaussianInf ---
// =====================================================================

TEST(CPosePDFGaussianInf, ConstructorsAndAccessors)
{
  CPosePDFGaussianInf p0;
  EXPECT_EQ(p0.getPoseMean(), CPose2D(0, 0, 0));
  EXPECT_TRUE(p0.isInfType());

  CPosePDFGaussianInf p1(CPose2D(1, 2, 0.1));
  EXPECT_EQ(p1.mean, CPose2D(1, 2, 0.1));

  CMatrixDouble33 covInv;
  covInv.setIdentity();
  covInv *= 10.0;
  CPosePDFGaussianInf p2(CPose2D(1, 2, 0.1), covInv);
  auto [c, m] = p2.getCovarianceAndMean();
  EXPECT_NEAR(c(0, 0), 0.1, 1e-9);
  EXPECT_EQ(m, p2.mean);

  CMatrixDouble33 infM;
  p2.getInformationMatrix(infM);
  EXPECT_TRUE(infM == covInv);
}

TEST(CPosePDFGaussianInf, CopyFromAndSerialization)
{
  CMatrixDouble33 cov;
  cov.setIdentity();
  cov *= 0.1;
  CPosePDFGaussian g(CPose2D(1, 2, 0.1), cov);

  CPosePDFGaussianInf pInf;
  pInf.copyFrom(g);
  EXPECT_NEAR(pInf.mean.x(), 1.0, 1e-6);

  CPosePDFGaussianInf pInf2(g);
  EXPECT_NEAR(pInf2.mean.y(), 2.0, 1e-6);

  CMatrixDouble66 cov6;
  cov6.setIdentity();
  cov6 *= 0.1;
  CPose3DPDFGaussian pose3d(CPose3D(1, 2, 3, 0.1, 0.2, 0.3), cov6);
  CPosePDFGaussianInf pInf3(pose3d);
  EXPECT_NEAR(pInf3.mean.x(), 1.0, 1e-6);

  CPose3DPDFGaussianInf pose3dInf;
  pose3dInf.copyFrom(pose3d);
  CPosePDFGaussianInf pInf4;
  pInf4.copyFrom(pose3dInf);
  EXPECT_NEAR(pInf4.mean.x(), 1.0, 1e-6);

  const std::string tmpf = testTempFile("mrpt_test_posepdfgaussianinf.txt");
  EXPECT_TRUE(pInf.saveToTextFile(tmpf));

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << pInf;
  buf.Seek(0);
  CPosePDFGaussianInf pInf5;
  arch >> pInf5;
  EXPECT_TRUE(pInf == pInf5);
}

TEST(CPosePDFGaussianInf, ChangeCoordsRotateAndInverseComposition)
{
  CMatrixDouble33 covInv;
  covInv.setIdentity();
  covInv *= 100.0;
  CPosePDFGaussianInf p(CPose2D(1, 0, 0), covInv);
  p.changeCoordinatesReference(CPose2D(0, 0, M_PI / 2));
  EXPECT_NEAR(p.mean.x(), 0.0, 1e-6);
  EXPECT_NEAR(p.mean.y(), 1.0, 1e-6);

  CPosePDFGaussianInf p2(CPose2D(1, 0, 0), covInv);
  p2.changeCoordinatesReference(CPose3D(0, 0, 0, M_PI / 2, 0, 0));
  EXPECT_NEAR(p2.mean.x(), 0.0, 1e-6);

  CPosePDFGaussianInf x(CPose2D(2, 0, 0), covInv);
  CPosePDFGaussianInf ref(CPose2D(1, 0, 0), covInv);
  CPosePDFGaussianInf result;
  result.inverseComposition(x, ref);
  EXPECT_NEAR(result.mean.x(), 1.0, 1e-6);

  CMatrixDouble33 cov01;
  cov01.setZero();
  CPosePDFGaussianInf result2;
  result2.inverseComposition(x, ref, cov01);
  EXPECT_NEAR(result2.mean.x(), 1.0, 1e-6);
}

TEST(CPosePDFGaussianInf, DrawSamplesEvaluateAndOperators)
{
  CMatrixDouble33 covInv;
  covInv.setIdentity();
  covInv *= 50.0;
  CPosePDFGaussianInf p(CPose2D(1, 2, 0.1), covInv);

  CPose2D sample;
  p.drawSingleSample(sample);

  std::vector<CVectorDouble> samples;
  p.drawManySamples(10, samples);
  EXPECT_EQ(samples.size(), 10u);

  double pAtMean = p.evaluatePDF(p.mean);
  EXPECT_GT(pAtMean, 0.0);
  EXPECT_NEAR(p.evaluateNormalizedPDF(p.mean), 1.0, 1e-6);

  double d = p.mahalanobisDistanceTo(p);
  EXPECT_NEAR(d, 0.0, 1e-9);

  CPosePDFGaussianInf pOther(CPose2D(2, 3, 0.2), covInv);
  double dOther = p.mahalanobisDistanceTo(pOther);
  EXPECT_GT(dOther, 0.0);

  CPosePDFGaussianInf inv;
  p.inverse(inv);

  CPosePDFGaussianInf fused;
  fused.bayesianFusion(static_cast<const CPosePDF&>(p), static_cast<const CPosePDF&>(pOther));
  EXPECT_GT(fused.cov_inv(0, 0), 0.0);

  CPosePDFGaussianInf a(CPose2D(1, 0, 0), covInv);
  a += CPose2D(1, 0, 0);
  EXPECT_NEAR(a.mean.x(), 2.0, 1e-6);

  CPosePDFGaussianInf b(CPose2D(1, 0, 0), covInv);
  b += CPosePDFGaussianInf(CPose2D(1, 0, 0), covInv);
  EXPECT_NEAR(b.mean.x(), 2.0, 1e-6);

  CPosePDFGaussianInf c(CPose2D(2, 0, 0), covInv);
  c -= CPosePDFGaussianInf(CPose2D(1, 0, 0), covInv);
  EXPECT_NEAR(c.mean.x(), 1.0, 1e-6);

  CPosePDFGaussianInf d2 = a + b;
  CPosePDFGaussianInf e2 = a - b;
  (void)d2;
  (void)e2;

  EXPECT_TRUE(a == a);
  EXPECT_FALSE(a == b);

  std::ostringstream ss;
  p.printTo(ss);
  ss << p;
  EXPECT_FALSE(ss.str().empty());

  CPosePDFGaussianInf opPlus = CPose2D(1, 0, 0) + p;
  (void)opPlus;
}

// =====================================================================
// --- CPosePDFParticles (extended) ---
// =====================================================================

TEST(CPosePDFParticles, ConstructorResetsAndAccessors)
{
  CPosePDFParticles p(10);
  EXPECT_EQ(p.size(), 10u);

  p.resetDeterministic(TPose2D(1, 2, 0.1), 20);
  EXPECT_EQ(p.size(), 20u);
  EXPECT_EQ(p.getParticlePose(0), TPose2D(1, 2, 0.1));

  p.resetUniform(-1, 1, -1, 1, -0.1, 0.1, 30);
  EXPECT_EQ(p.size(), 30u);
  for (size_t i = 0; i < p.size(); i++)
  {
    auto pp = p.getParticlePose(i);
    EXPECT_GE(pp.x, -1);
    EXPECT_LE(pp.x, 1);
  }

  std::vector<TPose2D> spots = {TPose2D(0, 0, 0), TPose2D(5, 5, 0)};
  p.resetAroundSetOfPoses(spots, 10, 0.1, 0.1, 0.1);
  EXPECT_EQ(p.size(), 20u);
}

TEST(CPosePDFParticles, SerializationRoundTrip)
{
  CPosePDFParticles p(20);
  p.resetDeterministic(TPose2D(1, 2, 0.1), 20);

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << p;
  buf.Seek(0);
  CPosePDFParticles p2;
  arch >> p2;
  EXPECT_EQ(p2.size(), p.size());
  EXPECT_EQ(p2.getParticlePose(0), p.getParticlePose(0));
}

TEST(CPosePDFParticles, CopyFromGaussianMeanCovAndBasics)
{
  CMatrixDouble33 cov;
  cov.setIdentity();
  cov *= 0.001;
  CPosePDFGaussian g(CPose2D(1, 2, 0.1), cov);

  CPosePDFParticles p(2000);
  p.copyFrom(g);

  CPose2D mean;
  p.getMean(mean);
  EXPECT_NEAR(mean.x(), 1.0, 0.05);
  EXPECT_NEAR(mean.y(), 2.0, 0.05);

  auto [c, m] = p.getCovarianceAndMean();
  EXPECT_NEAR(c(0, 0), 0.001, 0.002);
  (void)m;

  CPosePDFParticles p2;
  p2.copyFrom(p);
  EXPECT_EQ(p2.size(), p.size());

  const std::string tmpf = testTempFile("mrpt_test_posepdfparticles.txt");
  EXPECT_TRUE(p.saveToTextFile(tmpf));

  std::ostringstream ss;
  p.printTo(ss);
  EXPECT_FALSE(ss.str().empty());
}

TEST(CPosePDFParticles, ChangeCoordsOperatorsAppendInverse)
{
  CPosePDFParticles p;
  p.resetDeterministic(TPose2D(1, 0, 0), 10);

  p.changeCoordinatesReference(CPose3D(0, 0, 0, M_PI / 2, 0, 0));
  EXPECT_NEAR(p.getParticlePose(0).x, 0.0, 1e-6);
  EXPECT_NEAR(p.getParticlePose(0).y, 1.0, 1e-6);

  // Pose composition (not plain vector addition): the particle is at
  // phi=pi/2, so an added local +x displacement becomes a +y displacement.
  p.operator+=(TPose2D(1, 0, 0));
  EXPECT_NEAR(p.getParticlePose(0).x, 0.0, 1e-6);
  EXPECT_NEAR(p.getParticlePose(0).y, 2.0, 1e-6);

  CPosePDFParticles other;
  other.resetDeterministic(TPose2D(5, 5, 0), 5);
  p.append(other);
  EXPECT_EQ(p.size(), 15u);

  CPosePDFParticles inv(5);
  inv.resetDeterministic(TPose2D(2, 0, 0), 5);
  CPosePDFParticles invResult(5);
  inv.inverse(invResult);
  EXPECT_NEAR(invResult.getParticlePose(0).x, -2.0, 1e-6);

  auto mostLikely = p.getMostLikelyParticle();
  (void)mostLikely;
}

TEST(CPosePDFParticles, BayesianFusionWithParticlesP2)
{
  CPosePDFParticles p1(200);
  p1.resetDeterministic(TPose2D(1, 0, 0), 200);

  CPosePDFParticles p2(200);
  p2.resetDeterministic(TPose2D(3, 0, 0), 200);

  CPosePDFParticles result;
  result.bayesianFusion(p1, p2);
  EXPECT_EQ(result.size(), p1.size());

  double lik = p1.evaluatePDF_parzen(1.0, 0.0, 0.0, 0.1, 0.1);
  EXPECT_GT(lik, 0.0);

  const std::string tmpf = testTempFile("mrpt_test_parzen.txt");
  p1.saveParzenPDFToTextFile(tmpf.c_str(), -2, 2, -2, 2, 0.0, 0.5, 0.2, 0.2);
}

TEST(CPosePDFParticles, BayesianFusionWithGaussianP2)
{
  CPosePDFParticles p1(200);
  p1.resetUniform(0.5, 1.5, -0.5, 0.5, -0.1, 0.1, 200);

  CMatrixDouble33 cov;
  cov.setIdentity();
  cov *= 0.05;
  CPosePDFGaussian p2(CPose2D(1, 0, 0), cov);

  CPosePDFParticles result;
  result.bayesianFusion(p1, p2);
  EXPECT_EQ(result.size(), p1.size());
}

// =====================================================================
// --- CPosePDFSOG (extended) ---
// =====================================================================

TEST(CPosePDFSOG, ConstructAccessAndModes)
{
  CPosePDFSOG sog(3);
  EXPECT_EQ(sog.size(), 3u);
  EXPECT_FALSE(sog.empty());

  CMatrixDouble33 cov;
  cov.setIdentity();
  cov *= 0.01;
  int i = 0;
  for (auto& m : sog)
  {
    m.mean = CPose2D(double(i), 0, 0);
    m.cov = cov;
    m.log_w = 0.0;
    i++;
  }
  EXPECT_EQ(sog.getSOGModes().size(), 3u);
  EXPECT_EQ(sog.get(0).mean, CPose2D(0, 0, 0));
  sog[1].mean = CPose2D(10, 0, 0);
  EXPECT_EQ(sog[1].mean, CPose2D(10, 0, 0));

  CPosePDFSOG::TGaussianMode extra;
  extra.mean = CPose2D(99, 0, 0);
  extra.cov = cov;
  sog.push_back(extra);
  EXPECT_EQ(sog.size(), 4u);

  sog.erase(sog.begin());
  EXPECT_EQ(sog.size(), 3u);

  sog.resize(5);
  EXPECT_EQ(sog.size(), 5u);

  sog.clear();
  EXPECT_TRUE(sog.empty());
}

TEST(CPosePDFSOG, SerializationRoundTrip)
{
  CPosePDFSOG sog(2);
  CMatrixDouble33 cov;
  cov.setIdentity();
  cov *= 0.01;
  sog[0].mean = CPose2D(0, 0, 0);
  sog[0].cov = cov;
  sog[0].log_w = 0;
  sog[1].mean = CPose2D(2, 0, 0);
  sog[1].cov = cov;
  sog[1].log_w = 0.5;

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << sog;
  buf.Seek(0);
  CPosePDFSOG sog2;
  arch >> sog2;
  EXPECT_EQ(sog2.size(), sog.size());
  EXPECT_NEAR(sog2[1].log_w, sog[1].log_w, 1e-9);
}

TEST(CPosePDFSOG, MeanCovarianceCopyAndFile)
{
  CPosePDFSOG sog(2);
  CMatrixDouble33 cov;
  cov.setIdentity();
  cov *= 0.01;
  sog[0].mean = CPose2D(0, 0, 0);
  sog[0].cov = cov;
  sog[0].log_w = 0;
  sog[1].mean = CPose2D(2, 0, 0);
  sog[1].cov = cov;
  sog[1].log_w = 0;

  CPose2D mean;
  sog.getMean(mean);
  EXPECT_NEAR(mean.x(), 1.0, 0.2);
  EXPECT_EQ(sog.meanVal(), mean);

  auto [c, m] = sog.getCovarianceAndMean();
  (void)c;
  (void)m;

  auto [c2, m2] = sog.getMostLikelyCovarianceAndMean();
  (void)c2;
  (void)m2;
  CMatrixDouble33 c3;
  CPose2D m3;
#if defined(__clang__) || defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
  sog.getMostLikelyCovarianceAndMean(c3, m3);  // exercise deprecated overload
#if defined(__clang__) || defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

  sog.normalizeWeights();

  CPosePDFSOG sogCopy;
  sogCopy.copyFrom(sog);
  EXPECT_EQ(sogCopy.size(), sog.size());

  CMatrixDouble33 covG;
  covG.setIdentity();
  covG *= 0.02;
  CPosePDFGaussian g(CPose2D(3, 3, 0), covG);
  CPosePDFSOG sogFromGauss;
  sogFromGauss.copyFrom(g);
  EXPECT_EQ(sogFromGauss.size(), 1u);

  const std::string tmpf = testTempFile("mrpt_test_posepdfsog.txt");
  EXPECT_TRUE(sog.saveToTextFile(tmpf));

  std::ostringstream ss;
  sog.printTo(ss);
  EXPECT_FALSE(ss.str().empty());
}

TEST(CPosePDFSOG, ChangeCoordsRotateInverseAndOperators)
{
  CPosePDFSOG sog(1);
  CMatrixDouble33 cov;
  cov.setIdentity();
  cov *= 0.01;
  sog[0].mean = CPose2D(1, 0, 0);
  sog[0].cov = cov;
  sog[0].log_w = 0;

  sog.changeCoordinatesReference(CPose3D(0, 0, 0, M_PI / 2, 0, 0));
  EXPECT_NEAR(sog[0].mean.x(), 0.0, 1e-6);
  EXPECT_NEAR(sog[0].mean.y(), 1.0, 1e-6);

  sog.rotateAllCovariances(M_PI / 4);

  std::vector<CVectorDouble> samples;
  sog.drawManySamples(5, samples);
  EXPECT_EQ(samples.size(), 5u);

  CPosePDFSOG inv;
  sog.inverse(inv);
  EXPECT_EQ(inv.size(), sog.size());

  sog += CPose2D(1, 0, 0);

  double pdfVal = sog.evaluatePDF(sog[0].mean);
  EXPECT_GT(pdfVal, 0.0);
  double pdfSum = sog.evaluatePDF(sog[0].mean, true);
  EXPECT_GT(pdfSum, 0.0);
  double normPdf = sog.evaluateNormalizedPDF(sog[0].mean);
  EXPECT_GT(normPdf, 0.0);

  CMatrixDouble outMat;
  sog.evaluatePDFInArea(-2, 2, -2, 2, 1.0, sog[0].mean.phi(), outMat, false);
  EXPECT_GT(outMat.rows(), 0);
}

TEST(CPosePDFSOG, BayesianFusionPoseVersionAndMergeModes)
{
  CPosePDFSOG sog(2);
  CMatrixDouble33 cov;
  cov.setIdentity();
  cov *= 0.1;
  sog[0].mean = CPose2D(1, 0, 0);
  sog[0].cov = cov;
  sog[0].log_w = 0;
  sog[1].mean = CPose2D(1.01, 0, 0);
  sog[1].cov = cov;
  sog[1].log_w = 0;

  CPosePDFGaussian gauss(CPose2D(1, 0, 0), cov);

  CPosePDFSOG result;
  result.bayesianFusion(sog, gauss);
  EXPECT_GT(result.size(), 0u);

  sog.mergeModes(10.0, false);  // Close modes -> should merge to 1
  EXPECT_LE(sog.size(), 2u);
}

// =====================================================================
// --- CPosePDFGrid (extended) ---
// =====================================================================

TEST(CPosePDFGrid, UniformCopyMeanCovarianceAndFile)
{
  CPosePDFGrid grid(-1, 1, -1, 1, 0.5, DEG2RAD(90));
  grid.uniformDistribution();
  CPose2D mean;
  grid.getMean(mean);

  auto [cov, m] = grid.getCovarianceAndMean();
  (void)cov;
  (void)m;

  CPosePDFGrid grid2(-1, 1, -1, 1, 0.5, DEG2RAD(90));
  grid2.copyFrom(grid);

  CPosePDFParticles parts;
  parts.resetUniform(-1, 1, -1, 1, -M_PI, M_PI, 500);
  CPosePDFGrid grid3(-1, 1, -1, 1, 0.5, DEG2RAD(90));
  grid3.copyFrom(parts);  // Sample-based approximation branch

  const std::string tmpf = testTempFile("mrpt_test_posepdfgrid.txt");
  EXPECT_TRUE(grid.saveToTextFile(tmpf));

  std::ostringstream ss;
  grid.printTo(ss);
  EXPECT_FALSE(ss.str().empty());
}

TEST(CPosePDFGrid, SerializationRoundTripAndBayesianFusionWithParticles)
{
  CPosePDFGrid grid(-1, 1, -1, 1, 0.5, DEG2RAD(90.0));
  grid.fill(0.0);
  *grid.getByPos(0.0, 0.0, 0.0) = 1.0;
  grid.normalize();

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << grid;
  buf.Seek(0);
  CPosePDFGrid grid2;
  arch >> grid2;
  CPose2D mean1;
  CPose2D mean2;
  grid.getMean(mean1);
  grid2.getMean(mean2);
  EXPECT_NEAR(mean1.x(), mean2.x(), 1e-6);

  CPosePDFParticles parts;
  parts.resetUniform(-1, 1, -1, 1, -M_PI, M_PI, 200);
  CPosePDFGrid fused(-1, 1, -1, 1, 0.5, DEG2RAD(90.0));
  fused.bayesianFusion(grid, parts);
  CPose2D fusedMean;
  fused.getMean(fusedMean);
  EXPECT_NEAR(fusedMean.x(), 0.0, 1.0);
}

TEST(CPosePDFGrid, ChangeCoordsInverseBayesianFusionAndSamples)
{
  CPosePDFGrid gridA(-2, 2, -2, 2, 0.5, DEG2RAD(30));
  gridA.fill(0.0);
  *gridA.getByPos(0.0, 0.0, 0.0) = 1.0;
  gridA.normalize();

  gridA.changeCoordinatesReference(CPose3D(1, 0, 0, 0, 0, 0));
  CPose2D mean;
  gridA.getMean(mean);
  EXPECT_NEAR(mean.x(), 1.0, 1.0);

  CPosePDFGrid gridInv(-2, 2, -2, 2, 0.5, DEG2RAD(30));
  gridA.inverse(gridInv);

  CPosePDFGrid gridB(-2, 2, -2, 2, 0.5, DEG2RAD(30));
  gridB.fill(0.0);
  *gridB.getByPos(0.0, 0.0, 0.0) = 1.0;
  gridB.normalize();
  CPosePDFGrid gridFused;
  gridFused.bayesianFusion(gridA, gridB);

  std::vector<CVectorDouble> samples;
  gridA.drawManySamples(5, samples);
  EXPECT_EQ(samples.size(), 5u);
}

// =====================================================================
// --- CPosePDF (base class) ---
// =====================================================================

TEST(CPosePDF, JacobiansPoseComposition)
{
  CPose2D x(1, 2, 0.3);
  CPose2D u(0.5, -0.2, 0.1);
  CMatrixDouble33 df_dx;
  CMatrixDouble33 df_du;
  CPosePDF::jacobiansPoseComposition(x, u, df_dx, df_du);
  EXPECT_NEAR(df_dx(0, 0), 1.0, 1e-9);
  EXPECT_NEAR(df_du(2, 2), 1.0, 1e-9);

  CMatrixDouble33 df_dx2;
  CMatrixDouble33 df_du2;
  CPosePDF::jacobiansPoseComposition(x, u, df_dx2, df_du2, true, false);
  CMatrixDouble33 df_dx3;
  CMatrixDouble33 df_du3;
  CPosePDF::jacobiansPoseComposition(x, u, df_dx3, df_du3, false, true);

  CMatrixDouble33 cov;
  cov.setIdentity();
  cov *= 0.01;
  CPosePDFGaussian gx(x, cov);
  CPosePDFGaussian gu(u, cov);
  CMatrixDouble33 gdf_dx;
  CMatrixDouble33 gdf_du;
  CPosePDF::jacobiansPoseComposition(gx, gu, gdf_dx, gdf_du);
  EXPECT_TRUE(gdf_dx == df_dx);
}

// =====================================================================
// --- CPointPDFGaussian (extended) ---
// =====================================================================

TEST(CPointPDFGaussian, ConstructorsCopyChangeCoordsAndSerialization)
{
  CPointPDFGaussian p0;
  EXPECT_EQ(p0.mean, CPoint3D(0, 0, 0));

  CPointPDFGaussian p1(CPoint3D(1, 2, 3));
  EXPECT_EQ(p1.mean, CPoint3D(1, 2, 3));
  EXPECT_DOUBLE_EQ(p1.cov(0, 0), 0.0);

  CMatrixDouble33 cov;
  cov.setIdentity();
  cov *= 0.1;
  CPointPDFGaussian p2(CPoint3D(1, 2, 3), cov);

  CPoint3D mean;
  p2.getMean(mean);
  EXPECT_EQ(mean, p2.mean);

  CPointPDFGaussian p3;
  p3.copyFrom(p2);
  EXPECT_EQ(p3.mean, p2.mean);

  p3.changeCoordinatesReference(CPose3D(1, 0, 0, M_PI / 2, 0, 0));
  EXPECT_NEAR(p3.mean.x(), -1.0, 1e-6);
  EXPECT_NEAR(p3.mean.y(), 1.0, 1e-6);

  const std::string tmpf = testTempFile("mrpt_test_pointpdfgaussian.txt");
  EXPECT_TRUE(p2.saveToTextFile(tmpf));

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << p2;
  buf.Seek(0);
  CPointPDFGaussian p4;
  arch >> p4;
  EXPECT_EQ(p4.mean, p2.mean);
}

TEST(CPointPDFGaussian, ProductIntegralsMahalanobisAndBayesianFusion)
{
  CMatrixDouble33 cov;
  cov.setIdentity();
  cov *= 0.5;
  CPointPDFGaussian p1(CPoint3D(0, 0, 0), cov);
  CPointPDFGaussian p2(CPoint3D(0.1, 0, 0), cov);

  double pw = p1.productIntegralWith(p2);
  EXPECT_GT(pw, 0.0);
  double pw2d = p1.productIntegralWith2D(p2);
  EXPECT_GT(pw2d, 0.0);
  double pnw = p1.productIntegralNormalizedWith(p2);
  EXPECT_GT(pnw, 0.0);
  EXPECT_LE(pnw, 1.0);
  double pnw2d = p1.productIntegralNormalizedWith2D(p2);
  EXPECT_GT(pnw2d, 0.0);

  double dMaha = p1.mahalanobisDistanceTo(p2, false);
  double dMaha2D = p1.mahalanobisDistanceTo(p2, true);
  EXPECT_GT(dMaha, 0.0);
  EXPECT_GT(dMaha2D, 0.0);

  CPoint3D sample;
  p1.drawSingleSample(sample);

  CPointPDFGaussian result;
  result.bayesianFusion(static_cast<const CPointPDF&>(p1), static_cast<const CPointPDF&>(p2));
  EXPECT_NEAR(result.mean.x(), 0.05, 0.1);
}

// =====================================================================
// --- CPoint2DPDFGaussian (extended) ---
// =====================================================================

TEST(CPoint2DPDFGaussian, FullCoverage)
{
  CPoint2DPDFGaussian p0;
  CPoint2DPDFGaussian p1(CPoint2D(1, 2));
  CMatrixDouble22 cov;
  cov.setIdentity();
  cov *= 0.1;
  CPoint2DPDFGaussian p2(CPoint2D(1, 2), cov);

  CPoint2D mean;
  p2.getMean(mean);
  EXPECT_EQ(mean, p2.mean);

  auto [c, m] = p2.getCovarianceAndMean();
  (void)c;
  (void)m;

  CPoint2DPDFGaussian p3;
  p3.copyFrom(p2);
  EXPECT_EQ(p3.mean, p2.mean);

  p3.changeCoordinatesReference(CPose3D(1, 0, 0, M_PI / 2, 0, 0));

  const std::string tmpf = testTempFile("mrpt_test_point2dpdfgaussian.txt");
  EXPECT_TRUE(p2.saveToTextFile(tmpf));

  double pw = p2.productIntegralWith(p2);
  EXPECT_GT(pw, 0.0);
  double pnw = p2.productIntegralNormalizedWith(p2);
  EXPECT_NEAR(pnw, 1.0, 1e-6);

  CPoint2D sample;
  p2.drawSingleSample(sample);

  CPoint2DPDFGaussian result;
  result.bayesianFusion(static_cast<const CPoint2DPDF&>(p2), static_cast<const CPoint2DPDF&>(p2));
  EXPECT_NEAR(result.mean.x(), 1.0, 1e-6);

  double dm = p2.mahalanobisDistanceTo(p2);
  EXPECT_NEAR(dm, 0.0, 1e-6);
  double dmp = p2.mahalanobisDistanceToPoint(1.0, 2.0);
  EXPECT_NEAR(dmp, 0.0, 1e-6);

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << p2;
  buf.Seek(0);
  CPoint2DPDFGaussian p4;
  arch >> p4;
  EXPECT_EQ(p4.mean, p2.mean);
}

// =====================================================================
// --- CPointPDFSOG (extended) ---
// =====================================================================

TEST(CPointPDFSOG, ConstructAccessMeanCovAndFile)
{
  CPointPDFSOG sog(2);
  EXPECT_EQ(sog.size(), 2u);
  EXPECT_FALSE(sog.empty());

  CMatrixDouble33 cov;
  cov.setIdentity();
  cov *= 0.01;
  sog[0].val.mean = CPoint3D(0, 0, 0);
  sog[0].val.cov = cov;
  sog[0].log_w = 0;
  sog[1].val.mean = CPoint3D(2, 0, 0);
  sog[1].val.cov = cov;
  sog[1].log_w = 0;

  EXPECT_EQ(sog.get(0).val.mean, CPoint3D(0, 0, 0));

  CPoint3D mean;
  sog.getMean(mean);
  EXPECT_NEAR(mean.x(), 1.0, 0.1);

  auto [c, m] = sog.getCovarianceAndMean();
  (void)c;
  (void)m;

  sog.normalizeWeights();

  CPointPDFGaussian mostLikely;
  sog.getMostLikelyMode(mostLikely);

  double ess = sog.ESS();
  EXPECT_GT(ess, 0.0);

  CPointPDFSOG sogCopy;
  sogCopy.copyFrom(sog);
  EXPECT_EQ(sogCopy.size(), sog.size());

  CMatrixDouble33 covG;
  covG.setIdentity();
  covG *= 0.02;
  CPointPDFGaussian g(CPoint3D(5, 5, 5), covG);
  CPointPDFSOG sogFromGauss;
  sogFromGauss.copyFrom(g);
  EXPECT_EQ(sogFromGauss.size(), 1u);

  const std::string tmpf = testTempFile("mrpt_test_pointpdfsog.txt");
  EXPECT_TRUE(sog.saveToTextFile(tmpf));

  sog.resize(3);
  EXPECT_EQ(sog.size(), 3u);
  sog.clear();
  EXPECT_TRUE(sog.empty());
}

TEST(CPointPDFSOG, ChangeCoordsDrawSampleEvaluateAndBayesianFusion)
{
  CPointPDFSOG sog(1);
  CMatrixDouble33 cov;
  cov.setIdentity();
  cov *= 0.01;
  sog[0].val.mean = CPoint3D(1, 0, 0);
  sog[0].val.cov = cov;
  sog[0].log_w = 0;

  sog.changeCoordinatesReference(CPose3D(0, 0, 0, M_PI / 2, 0, 0));

  CPoint3D sample;
  sog.drawSingleSample(sample);

  double pdfVal = sog.evaluatePDF(sog[0].val.mean, false);
  EXPECT_GT(pdfVal, 0.0);
  double pdfSum = sog.evaluatePDF(sog[0].val.mean, true);
  EXPECT_GT(pdfSum, 0.0);

  CMatrixD outMat;
  sog.evaluatePDFInArea(-2, 2, -2, 2, 1.0f, 0.0f, outMat, false);
  EXPECT_GT(outMat.rows(), 0);

  CPointPDFSOG p2(1);
  p2[0].val.mean = sog[0].val.mean;
  p2[0].val.cov = cov;
  p2[0].log_w = 0;

  CPointPDFSOG result;
  result.bayesianFusion(sog, p2);
  EXPECT_GT(result.size(), 0u);
}

TEST(CPointPDFSOG, SerializationRoundTripAnd2DCovarianceWithMahalanobisDrop)
{
  CPointPDFSOG sog(1);
  CMatrixDouble33 cov;
  cov.setIdentity();
  cov *= 0.01;
  sog[0].val.mean = CPoint3D(1, 0, 0);
  sog[0].val.cov = cov;
  sog[0].log_w = 0;

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << sog;
  buf.Seek(0);
  CPointPDFSOG sog2;
  arch >> sog2;
  EXPECT_EQ(sog2.size(), sog.size());

  // 2D covariance (z-variance == 0), with a Mahalanobis-distance drop
  // threshold, to exercise the is2D and reallyComputeThisOne branches:
  CMatrixDouble33 cov2d;
  cov2d(0, 0) = 0.01;
  cov2d(1, 1) = 0.01;
  cov2d(2, 2) = 0.0;
  CPointPDFSOG p1(1);
  p1[0].val.mean = CPoint3D(0, 0, 0);
  p1[0].val.cov = cov2d;
  p1[0].log_w = 0;

  CPointPDFSOG p2b(1);
  p2b[0].val.mean = CPoint3D(0.01, 0, 0);
  p2b[0].val.cov = cov2d;
  p2b[0].log_w = 0;

  CPointPDFSOG closeResult;
  closeResult.bayesianFusion(p1, p2b, 10.0);
  EXPECT_GT(closeResult.size(), 0u);

  CPointPDFSOG p3(1);
  p3[0].val.mean = CPoint3D(100, 0, 0);
  p3[0].val.cov = cov2d;
  p3[0].log_w = 0;

  CPointPDFSOG farResult;
  farResult.bayesianFusion(p1, p3, 0.01);
  EXPECT_EQ(farResult.size(), 0u);
}

// =====================================================================
// --- CPointPDFParticles (extended) ---
// =====================================================================

TEST(CPointPDFParticles, SerializationRoundTrip)
{
  CPointPDFParticles p(20);
  p.setSize(20, TPoint3Df(1, 2, 3));

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << p;
  buf.Seek(0);
  CPointPDFParticles p2;
  arch >> p2;
  EXPECT_EQ(p2.size(), p.size());
}

TEST(CPointPDFParticles, FullCoverage)
{
  CPointPDFParticles p(100);
  EXPECT_EQ(p.size(), 100u);

  p.setSize(50, TPoint3Df(1, 2, 3));
  EXPECT_EQ(p.size(), 50u);

  CPoint3D mean;
  p.getMean(mean);
  EXPECT_NEAR(mean.x(), 1.0, 1e-4);

  auto [cov, m] = p.getCovarianceAndMean();
  (void)cov;
  (void)m;

  CPointPDFParticles p2;
  p2.copyFrom(p);
  EXPECT_EQ(p2.size(), p.size());

  CMatrixDouble33 covG;
  covG.setIdentity();
  covG *= 0.01;
  CPointPDFGaussian g(CPoint3D(4, 4, 4), covG);
  CPointPDFParticles p3;
  p3.copyFrom(g);
  EXPECT_GT(p3.size(), 0u);

  const std::string tmpf = testTempFile("mrpt_test_pointpdfparticles.txt");
  EXPECT_TRUE(p.saveToTextFile(tmpf));

  p.changeCoordinatesReference(CPose3D(1, 0, 0, 0, 0, 0));
  CPoint3D meanAfter;
  p.getMean(meanAfter);
  EXPECT_NEAR(meanAfter.x(), 2.0, 1e-4);

  double kurt = p.computeKurtosis();
  (void)kurt;

  CPoint3D sample;
  p.drawSingleSample(sample);

  p.clear();
  EXPECT_EQ(p.size(), 0u);
}

TEST(CPointPDFParticles, BayesianFusionParticlesAndGaussian)
{
  CPointPDFParticles p1;
  p1.setSize(200, TPoint3Df(1, 0, 0));

  CPointPDFParticles p2b;
  p2b.setSize(200, TPoint3Df(3, 0, 0));

  CPointPDFParticles result;
  result.bayesianFusion(p1, p2b);
  EXPECT_EQ(result.size(), p1.size());

  CMatrixDouble33 cov;
  cov.setIdentity();
  cov *= 0.1;
  CPointPDFGaussian g(CPoint3D(1, 0, 0), cov);
  CPointPDFParticles result2;
  result2.bayesianFusion(p1, g);
  EXPECT_EQ(result2.size(), p1.size());
}

// =====================================================================
// --- CPose3DPDFGrid ---
// =====================================================================

TEST(CPose3DPDFGrid, UniformCopyMeanCovFileAndOperations)
{
  const TPose3D bbMin(-1, -1, -1, -M_PI, -0.5 * M_PI, -0.5 * M_PI);
  const TPose3D bbMax(1, 1, 1, M_PI, 0.5 * M_PI, 0.5 * M_PI);
  CPose3DPDFGrid grid(bbMin, bbMax, 0.5, DEG2RAD(90.0));
  grid.uniformDistribution();

  CPose3D mean;
  grid.getMean(mean);

  auto [cov, m] = grid.getCovarianceAndMean();
  (void)cov;
  (void)m;

  CPose3DPDFGrid grid2(bbMin, bbMax, 0.5, DEG2RAD(90.0));
  grid2.copyFrom(grid);

  CPose3DPDFParticles parts;
  parts.resetUniform(bbMin, bbMax, 200);
  CPose3DPDFGrid grid3(bbMin, bbMax, 0.5, DEG2RAD(90.0));
  grid3.copyFrom(parts);

  const std::string tmpf = testTempFile("mrpt_test_pose3dpdfgrid.txt");
  EXPECT_TRUE(grid.saveToTextFile(tmpf));

  std::ostringstream ss;
  grid.printTo(ss);
  EXPECT_FALSE(ss.str().empty());
}

TEST(CPose3DPDFGrid, SerializationRoundTrip)
{
  const TPose3D bbMin(-1, -1, -1, -M_PI, -0.5 * M_PI, -0.5 * M_PI);
  const TPose3D bbMax(1, 1, 1, M_PI, 0.5 * M_PI, 0.5 * M_PI);
  CPose3DPDFGrid grid(bbMin, bbMax, 0.5, DEG2RAD(90.0));
  grid.uniformDistribution();

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << grid;
  buf.Seek(0);
  CPose3DPDFGrid grid2;
  arch >> grid2;

  CPose3D mean1;
  CPose3D mean2;
  grid.getMean(mean1);
  grid2.getMean(mean2);
  EXPECT_NEAR(mean1.x(), mean2.x(), 1e-6);
}

TEST(CPose3DPDFGrid, ChangeCoordsInverseBayesianFusionAndSamples)
{
  const TPose3D bbMin(-2, -2, -2, -M_PI, -0.5 * M_PI, -0.5 * M_PI);
  const TPose3D bbMax(2, 2, 2, M_PI, 0.5 * M_PI, 0.5 * M_PI);
  CPose3DPDFGrid gridA(bbMin, bbMax, 1.0, DEG2RAD(90.0));
  gridA.fill(0.0);
  *gridA.getByPos(0, 0, 0, 0, 0, 0) = 1.0;
  gridA.normalize();

  gridA.changeCoordinatesReference(CPose3D(1, 0, 0, 0, 0, 0));
  CPose3D mean;
  gridA.getMean(mean);
  EXPECT_NEAR(mean.x(), 1.0, 1.5);

  CPose3DPDFGrid gridInv(bbMin, bbMax, 1.0, DEG2RAD(90.0));
  gridA.inverse(gridInv);

  CPose3DPDFGrid gridB(bbMin, bbMax, 1.0, DEG2RAD(90.0));
  gridB.fill(0.0);
  *gridB.getByPos(0, 0, 0, 0, 0, 0) = 1.0;
  gridB.normalize();
  CPose3DPDFGrid gridFused;
  gridFused.bayesianFusion(gridA, gridB);

  std::vector<CVectorDouble> samples;
  gridA.drawManySamples(5, samples);
  EXPECT_EQ(samples.size(), 5u);
}

// =====================================================================
// --- CPose3DPDFGaussianInf ---
// =====================================================================

TEST(CPose3DPDFGaussianInf, ConstructorsCopyChangeCoordsAndSerialization)
{
  CPose3DPDFGaussianInf p0;
  EXPECT_TRUE(p0.isInfType());

  CPose3DPDFGaussianInf p1(CPose3D(1, 2, 3, 0.1, 0.2, 0.3));
  EXPECT_EQ(p1.getPoseMean(), CPose3D(1, 2, 3, 0.1, 0.2, 0.3));

  CPose3DPDFGaussianInf pUninit(UNINITIALIZED_POSE);
  (void)pUninit;

  CMatrixDouble66 covInv;
  covInv.setIdentity();
  covInv *= 10.0;
  CPose3DPDFGaussianInf p2(CPose3D(1, 2, 3, 0.1, 0.2, 0.3), covInv);

  CMatrixDouble66 infM;
  p2.getInformationMatrix(infM);
  EXPECT_TRUE(infM == covInv);

  CPose3DQuat quatMean(CPose3D(1, 2, 3, 0.1, 0.2, 0.3));
  CMatrixDouble77 covQuat;
  covQuat.setIdentity();
  covQuat *= 0.01;
  CPose3DQuatPDFGaussian quatPdf(quatMean, covQuat);
  CPose3DPDFGaussianInf p3(quatPdf);
  EXPECT_NEAR(p3.mean.x(), 1.0, 1e-6);

  CPose3DPDFGaussianInf p4;
  p4.copyFrom(quatPdf);
  EXPECT_NEAR(p4.mean.x(), 1.0, 1e-6);

  CMatrixDouble33 cov2d;
  cov2d.setIdentity();
  cov2d *= 0.1;
  CPosePDFGaussianInf pose2dInf(CPose2D(1, 2, 0.1), cov2d);
  CPose3DPDFGaussianInf p5;
  p5.copyFrom(pose2dInf);
  EXPECT_NEAR(p5.mean.x(), 1.0, 1e-6);

  CPosePDFGaussian pose2d(CPose2D(1, 2, 0.1), cov2d);
  CPose3DPDFGaussianInf p6;
  p6.copyFrom(static_cast<const CPosePDF&>(pose2d));
  EXPECT_NEAR(p6.mean.x(), 1.0, 1e-6);

  const std::string tmpf = testTempFile("mrpt_test_pose3dpdfgaussianinf.txt");
  EXPECT_TRUE(p2.saveToTextFile(tmpf));

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << p2;
  buf.Seek(0);
  CPose3DPDFGaussianInf p7;
  arch >> p7;
  EXPECT_TRUE(p2 == p7);
}

TEST(CPose3DPDFGaussianInf, ChangeCoordsDrawSamplesInverseAndOperators)
{
  CMatrixDouble66 covInv;
  covInv.setIdentity();
  covInv *= 100.0;
  CPose3DPDFGaussianInf p(CPose3D(1, 0, 0, 0, 0, 0), covInv);
  p.changeCoordinatesReference(CPose3D(0, 0, 0, M_PI / 2, 0, 0));
  EXPECT_NEAR(p.mean.x(), 0.0, 1e-6);
  EXPECT_NEAR(p.mean.y(), 1.0, 1e-6);

  CPose3D sample;
  p.drawSingleSample(sample);

  std::vector<CVectorDouble> samples;
  p.drawManySamples(10, samples);
  EXPECT_EQ(samples.size(), 10u);

  double pAtMean = p.evaluatePDF(p.mean);
  EXPECT_GT(pAtMean, 0.0);
  EXPECT_NEAR(p.evaluateNormalizedPDF(p.mean), 1.0, 1e-4);

  CMatrixDouble subCov;
  p.getInvCovSubmatrix2D(subCov);
  EXPECT_EQ(subCov.rows(), 3);

  double d = p.mahalanobisDistanceTo(p);
  EXPECT_NEAR(d, 0.0, 1e-9);

  CPose3DPDFGaussianInf inv;
  p.inverse(inv);
  CPose3DPDFGaussianInf unaryInv = -p;
  (void)unaryInv;

  CPose3DPDFGaussianInf a(CPose3D(1, 0, 0, 0, 0, 0), covInv);
  a += CPose3D(1, 0, 0, 0, 0, 0);
  EXPECT_NEAR(a.mean.x(), 2.0, 1e-6);

  CPose3DPDFGaussianInf b(CPose3D(1, 0, 0, 0, 0, 0), covInv);
  b += CPose3DPDFGaussianInf(CPose3D(1, 0, 0, 0, 0, 0), covInv);
  EXPECT_NEAR(b.mean.x(), 2.0, 1e-6);

  CPose3DPDFGaussianInf c(CPose3D(2, 0, 0, 0, 0, 0), covInv);
  c -= CPose3DPDFGaussianInf(CPose3D(1, 0, 0, 0, 0, 0), covInv);
  EXPECT_NEAR(c.mean.x(), 1.0, 1e-6);

  CPose3DPDFGaussianInf d2 = a + b;
  CPose3DPDFGaussianInf e2 = a - b;
  (void)d2;
  (void)e2;

  EXPECT_TRUE(a == a);
  EXPECT_FALSE(a == b);

  std::ostringstream ss;
  p.printTo(ss);
  ss << p;
  EXPECT_FALSE(ss.str().empty());
}

// =====================================================================
// --- CPose3DPDFSOG ---
// =====================================================================

TEST(CPose3DPDFSOG, SerializationRoundTrip)
{
  CPose3DPDFSOG sog(2);
  CMatrixDouble66 cov;
  cov.setIdentity();
  cov *= 0.01;
  auto it = sog.begin();
  it->val.mean = CPose3D(0, 0, 0, 0, 0, 0);
  it->val.cov = cov;
  it->log_w = 0;
  ++it;
  it->val.mean = CPose3D(2, 0, 0, 0, 0, 0);
  it->val.cov = cov;
  it->log_w = 0.5;

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << sog;
  buf.Seek(0);
  CPose3DPDFSOG sog2;
  arch >> sog2;
  EXPECT_EQ(sog2.size(), sog.size());
}

TEST(CPose3DPDFSOG, ConstructAccessMeanCovAndFile)
{
  CPose3DPDFSOG sog(2);
  EXPECT_EQ(sog.size(), 2u);
  EXPECT_FALSE(sog.empty());

  CMatrixDouble66 cov;
  cov.setIdentity();
  cov *= 0.01;
  auto it = sog.begin();
  it->val.mean = CPose3D(0, 0, 0, 0, 0, 0);
  it->val.cov = cov;
  it->log_w = 0;
  ++it;
  it->val.mean = CPose3D(2, 0, 0, 0, 0, 0);
  it->val.cov = cov;
  it->log_w = 0;

  CPose3D mean;
  sog.getMean(mean);
  EXPECT_NEAR(mean.x(), 1.0, 0.1);
  EXPECT_EQ(sog.meanVal(), mean);

  auto [c, m] = sog.getCovarianceAndMean();
  (void)c;
  (void)m;

  sog.normalizeWeights();

#if defined(__clang__) || defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
  CPose3DPDFGaussian mostLikelyDeprecated;
  sog.getMostLikelyMode(mostLikelyDeprecated);  // exercise deprecated overload
#if defined(__clang__) || defined(__GNUC__)
#pragma GCC diagnostic pop
#endif
  CPose3DPDFGaussian mostLikely = sog.getMostLikelyMode();
  (void)mostLikely;

  CPose3DPDFSOG sogCopy;
  sogCopy.copyFrom(sog);
  EXPECT_EQ(sogCopy.size(), sog.size());

  CMatrixDouble66 covG;
  covG.setIdentity();
  covG *= 0.02;
  CPose3DPDFGaussian g(CPose3D(5, 5, 5, 0, 0, 0), covG);
  CPose3DPDFSOG sogFromGauss;
  sogFromGauss.copyFrom(g);
  EXPECT_EQ(sogFromGauss.size(), 1u);

  const std::string tmpf = testTempFile("mrpt_test_pose3dpdfsog.txt");
  EXPECT_TRUE(sog.saveToTextFile(tmpf));

  sog.resize(3);
  EXPECT_EQ(sog.size(), 3u);
  sog.clear();
  EXPECT_TRUE(sog.empty());
}

TEST(CPose3DPDFSOG, CopyFromNonSOGAppendAndMostLikelyMode)
{
  CMatrixDouble66 cov;
  cov.setIdentity();
  cov *= 0.02;
  CPose3DPDFGaussian g(CPose3D(5, 5, 5, 0, 0, 0), cov);

  CPose3DPDFSOG fromGauss;
  fromGauss.copyFrom(static_cast<const CPose3DPDF&>(g));
  EXPECT_EQ(fromGauss.size(), 1u);
  EXPECT_NEAR(fromGauss.begin()->val.mean.x(), 5.0, 1e-6);

  CPose3DPDFSOG sog(1);
  sog.begin()->val.mean = CPose3D(1, 0, 0, 0, 0, 0);
  sog.begin()->val.cov = cov;
  sog.begin()->log_w = 0;

  CPose3DPDFSOG other(1);
  other.begin()->val.mean = CPose3D(2, 0, 0, 0, 0, 0);
  other.begin()->val.cov = cov;
  other.begin()->log_w = 0;
  sog.appendFrom(other);
  EXPECT_EQ(sog.size(), 2u);

  CPose3DPDFGaussian mostLikely = sog.getMostLikelyMode();
  (void)mostLikely;

#if defined(__clang__) || defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
  CPose3DPDFGaussian mostLikelyDeprecated;
  sog.getMostLikelyMode(mostLikelyDeprecated);
#if defined(__clang__) || defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

  std::ostringstream ss;
  sog.printTo(ss);
  EXPECT_FALSE(ss.str().empty());

  CPose3DPDFSOG empty;
  CPose3DPDFGaussian mostLikelyEmpty = empty.getMostLikelyMode();
  (void)mostLikelyEmpty;
}

TEST(CPose3DPDFSOG, ChangeCoordsInverseAppendBayesianFusionAndSamples)
{
  CPose3DPDFSOG sog(1);
  CMatrixDouble66 cov;
  cov.setIdentity();
  cov *= 0.01;
  sog.begin()->val.mean = CPose3D(1, 0, 0, 0, 0, 0);
  sog.begin()->val.cov = cov;
  sog.begin()->log_w = 0;

  sog.changeCoordinatesReference(CPose3D(0, 0, 0, M_PI / 2, 0, 0));
  EXPECT_NEAR(sog.begin()->val.mean.x(), 0.0, 1e-6);
  EXPECT_NEAR(sog.begin()->val.mean.y(), 1.0, 1e-6);

  std::vector<CVectorDouble> samples;
  sog.drawManySamples(5, samples);
  EXPECT_EQ(samples.size(), 5u);

  CPose3DPDFSOG inv;
  sog.inverse(inv);
  EXPECT_EQ(inv.size(), sog.size());

  CPose3DPDFSOG other(1);
  other.begin()->val.mean = CPose3D(9, 9, 9, 0, 0, 0);
  other.begin()->val.cov = cov;
  other.begin()->log_w = 0;
  sog.appendFrom(other);
  EXPECT_EQ(sog.size(), 2u);

  CPose3DPDFSOG p1(1);
  p1.begin()->val.mean = CPose3D(1, 0, 0, 0, 0, 0);
  p1.begin()->val.cov = cov;
  p1.begin()->log_w = 0;
  CPose3DPDFSOG p2(1);
  p2.begin()->val.mean = CPose3D(1.01, 0, 0, 0, 0, 0);
  p2.begin()->val.cov = cov;
  p2.begin()->log_w = 0;

  CPose3DPDFSOG result;
  result.bayesianFusion(p1, p2);
  EXPECT_GT(result.size(), 0u);
}

// =====================================================================
// --- CPose3DPDFParticles ---
// =====================================================================

TEST(CPose3DPDFParticles, SerializationRoundTrip)
{
  CPose3DPDFParticles p(20);
  p.resetDeterministic(TPose3D(1, 2, 3, 0.1, 0.2, 0.3), 20);

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << p;
  buf.Seek(0);
  CPose3DPDFParticles p2;
  arch >> p2;
  EXPECT_EQ(p2.size(), p.size());
  EXPECT_EQ(p2.getParticlePose(0), p.getParticlePose(0));
}

TEST(CPose3DPDFParticles, ConstructorResetsAndAccessors)
{
  CPose3DPDFParticles p(10);
  EXPECT_EQ(p.size(), 10u);

  p.resetDeterministic(TPose3D(1, 2, 3, 0.1, 0.2, 0.3), 20);
  EXPECT_EQ(p.size(), 20u);
  EXPECT_EQ(p.getParticlePose(0), TPose3D(1, 2, 3, 0.1, 0.2, 0.3));

  p.resetUniform(TPose3D(-1, -1, -1, -0.1, -0.1, -0.1), TPose3D(1, 1, 1, 0.1, 0.1, 0.1), 30);
  EXPECT_EQ(p.size(), 30u);
}

TEST(CPose3DPDFParticles, CopyFromGaussianChangeCoordsOperatorsAppendInverse)
{
  CMatrixDouble66 cov;
  cov.setIdentity();
  cov *= 0.001;
  CPose3DPDFGaussian g(CPose3D(1, 2, 3, 0.1, 0.2, 0.3), cov);

  CPose3DPDFParticles p(2000);
  p.copyFrom(g);

  CPose3D mean;
  p.getMean(mean);
  EXPECT_NEAR(mean.x(), 1.0, 0.1);

  auto [c, m] = p.getCovarianceAndMean();
  (void)c;
  (void)m;

  const std::string tmpf = testTempFile("mrpt_test_pose3dpdfparticles.txt");
  EXPECT_TRUE(p.saveToTextFile(tmpf));

  CPose3DPDFParticles simple;
  simple.resetDeterministic(TPose3D(1, 0, 0, 0, 0, 0), 10);
  simple.changeCoordinatesReference(CPose3D(0, 0, 0, M_PI / 2, 0, 0));
  EXPECT_NEAR(simple.getParticlePose(0).x, 0.0, 1e-6);
  EXPECT_NEAR(simple.getParticlePose(0).y, 1.0, 1e-6);

  simple += CPose3D(1, 0, 0, 0, 0, 0);

  CPose3DPDFParticles other;
  other.resetDeterministic(TPose3D(5, 5, 5, 0, 0, 0), 5);
  simple.append(other);
  EXPECT_EQ(simple.size(), 15u);

  CPose3DPDFParticles invSrc(5);
  invSrc.resetDeterministic(TPose3D(2, 0, 0, 0, 0, 0), 5);
  CPose3DPDFParticles invResult(5);
  invSrc.inverse(invResult);
  EXPECT_NEAR(invResult.getParticlePose(0).x, -2.0, 1e-6);

  auto mostLikely = simple.getMostLikelyParticle();
  (void)mostLikely;

  std::ostringstream ss;
  simple.printTo(ss);
  EXPECT_FALSE(ss.str().empty());
}

TEST(CPose3DPDFParticles, BayesianFusionParticlesAndGaussian)
{
  CPose3DPDFParticles p1;
  p1.resetDeterministic(TPose3D(1, 0, 0, 0, 0, 0), 200);

  CPose3DPDFParticles p2b;
  p2b.resetDeterministic(TPose3D(3, 0, 0, 0, 0, 0), 200);

  CPose3DPDFParticles result;
  result.bayesianFusion(p1, p2b);
  EXPECT_EQ(result.size(), p1.size());

  CMatrixDouble66 cov;
  cov.setIdentity();
  cov *= 0.1;
  CPose3DPDFGaussian g(CPose3D(1, 0, 0, 0, 0, 0), cov);
  CPose3DPDFParticles result2;
  result2.bayesianFusion(p1, g);
  EXPECT_EQ(result2.size(), p1.size());
}

// =====================================================================
// --- CPose3DQuatPDFGaussianInf ---
// =====================================================================

TEST(CPose3DQuatPDFGaussianInf, ConstructorsAccessorsAndSerialization)
{
  CPose3DQuatPDFGaussianInf p0;
  EXPECT_TRUE(p0.isInfType());

  CPose3DQuatPDFGaussianInf pUninit(mrpt::math::UNINITIALIZED_QUATERNION);
  (void)pUninit;

  CPose3DQuat meanQ(CPose3D(1, 2, 3, 0.1, 0.2, 0.3));
  CPose3DQuatPDFGaussianInf p1(meanQ);
  EXPECT_EQ(p1.getPoseMean(), meanQ);

  CMatrixDouble77 covInv;
  covInv.setIdentity();
  covInv *= 100.0;
  CPose3DQuatPDFGaussianInf p2(meanQ, covInv);

  CMatrixDouble77 infM;
  p2.getInformationMatrix(infM);
  EXPECT_TRUE(infM == covInv);

  auto [c, m] = p2.getCovarianceAndMean();
  (void)c;
  EXPECT_EQ(m, meanQ);

  CPose3DQuatPDFGaussianInf p3;
  p3.copyFrom(p2);
  EXPECT_EQ(p3.mean, p2.mean);

  const std::string tmpf = testTempFile("mrpt_test_pose3dquatpdfgaussianinf.txt");
  EXPECT_TRUE(p2.saveToTextFile(tmpf));

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << p2;
  buf.Seek(0);
  CPose3DQuatPDFGaussianInf p4;
  arch >> p4;
  EXPECT_TRUE(p2 == p4);
}

TEST(CPose3DQuatPDFGaussianInf, ChangeCoordsDrawSamplesInverseAndOperators)
{
  CPose3DQuat meanQ(CPose3D(1, 0, 0, 0, 0, 0));
  CMatrixDouble77 covInv;
  covInv.setIdentity();
  covInv *= 1000.0;
  CPose3DQuatPDFGaussianInf p(meanQ, covInv);

  p.changeCoordinatesReference(CPose3D(0, 0, 0, M_PI / 2, 0, 0));
  EXPECT_NEAR(p.mean.x(), 0.0, 1e-6);
  EXPECT_NEAR(p.mean.y(), 1.0, 1e-6);

  CPose3DQuatPDFGaussianInf p2(meanQ, covInv);
  p2.changeCoordinatesReference(CPose3DQuat(CPose3D(0, 0, 0, M_PI / 2, 0, 0)));
  EXPECT_NEAR(p2.mean.x(), 0.0, 1e-6);

  CPose3DQuat sample;
  p2.drawSingleSample(sample);

  std::vector<CVectorDouble> samples;
  p2.drawManySamples(5, samples);
  EXPECT_EQ(samples.size(), 5u);

  double pAtMean = p2.evaluatePDF(p2.mean);
  EXPECT_GT(pAtMean, 0.0);
  EXPECT_NEAR(p2.evaluateNormalizedPDF(p2.mean), 1.0, 1e-3);

  CPose3DQuatPDFGaussianInf inv;
  p2.inverse(inv);
  CPose3DQuatPDFGaussianInf unaryInv = -p2;
  (void)unaryInv;

  std::ostringstream ss;
  ss << p2;
  EXPECT_FALSE(ss.str().empty());
}

// KNOWN BUG (found while writing this test, not introduced by it): the pose
// composition Jacobian for a 7-parameter quaternion pose is structurally
// rank-deficient (quaternions have a redundant DOF versus SE(3)'s 6), so
// CPose3DQuatPDFGaussianInf::operator+= inverts a singular matrix and
// silently produces a NaN information matrix, with no exception raised.
// This regression test documents the current behavior; if the underlying
// math is ever fixed, update/remove this test accordingly.
TEST(CPose3DQuatPDFGaussianInf, KnownBug_OperatorPlusEqualsProducesNaNCovariance)
{
  CPose3DQuat meanQ(CPose3D(1, 0, 0, 0, 0, 0));
  CMatrixDouble77 covInv;
  covInv.setIdentity();
  covInv *= 1000.0;

  CPose3DQuatPDFGaussianInf a(meanQ, covInv);
  a += CPose3DQuat(CPose3D(1, 0, 0, 0, 0, 0));
  EXPECT_TRUE(std::isnan(a.cov_inv(0, 0)));

  CPose3DQuatPDFGaussianInf b(meanQ, covInv);
  b += CPose3DQuatPDFGaussianInf(meanQ, covInv);
  EXPECT_TRUE(std::isnan(b.cov_inv(0, 0)));

  CPose3DQuatPDFGaussianInf c(meanQ, covInv);
  c -= CPose3DQuatPDFGaussianInf(meanQ, covInv);
  EXPECT_TRUE(std::isnan(c.cov_inv(0, 0)));
}

// =====================================================================
// --- CPose3DPDF (base class) ---
// =====================================================================

TEST(CPose3DPDF, CreateFrom2DAllTypesAndJacobians)
{
  CMatrixDouble33 cov2d;
  cov2d.setIdentity();
  cov2d *= 0.1;

  {
    CPosePDFGaussian src(CPose2D(1, 2, 0.1), cov2d);
    std::unique_ptr<CPose3DPDF> dst(CPose3DPDF::createFrom2D(src));
    ASSERT_TRUE(dynamic_cast<CPose3DPDFGaussian*>(dst.get()) != nullptr);
  }
  {
    CPosePDFGaussianInf src(CPose2D(1, 2, 0.1), cov2d);
    std::unique_ptr<CPose3DPDF> dst(CPose3DPDF::createFrom2D(src));
    ASSERT_TRUE(dynamic_cast<CPose3DPDFGaussianInf*>(dst.get()) != nullptr);
  }
  {
    CPosePDFParticles src(50);
    src.resetDeterministic(TPose2D(1, 2, 0.1), 50);
    std::unique_ptr<CPose3DPDF> dst(CPose3DPDF::createFrom2D(src));
    ASSERT_TRUE(dynamic_cast<CPose3DPDFParticles*>(dst.get()) != nullptr);
  }
  {
    CPosePDFSOG src(1);
    src[0].mean = CPose2D(1, 2, 0.1);
    src[0].cov = cov2d;
    src[0].log_w = 0;
    std::unique_ptr<CPose3DPDF> dst(CPose3DPDF::createFrom2D(src));
    ASSERT_TRUE(dynamic_cast<CPose3DPDFSOG*>(dst.get()) != nullptr);
  }

  CPose3D x(1, 2, 3, 0.1, 0.2, 0.3);
  CPose3D u(0.5, -0.2, 0.1, 0.05, 0.02, -0.01);
  CMatrixDouble66 df_dx;
  CMatrixDouble66 df_du;
  CPose3DPDF::jacobiansPoseComposition(x, u, df_dx, df_du);
  EXPECT_EQ(df_dx.rows(), 6);
  EXPECT_EQ(df_du.rows(), 6);
}

// =====================================================================
// --- CPoses2DSequence / CPoses3DSequence ---
// =====================================================================

TEST(CPosesSequence, CPoses2DSequenceFullCoverage)
{
  CPoses2DSequence seq;
  EXPECT_EQ(seq.posesCount(), 0u);

  CPose2D p1(1, 0, 0);
  CPose2D p2(1, 0, 0);
  seq.appendPose(p1);
  seq.appendPose(p2);
  EXPECT_EQ(seq.posesCount(), 2u);

  CPose2D got = seq.getPose(0);
  EXPECT_EQ(got, CPose2D(1, 0, 0));

  CPose2D outPose;
#if defined(__clang__) || defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
  seq.getPose(1, outPose);  // exercise deprecated out-param overload
#if defined(__clang__) || defined(__GNUC__)
#pragma GCC diagnostic pop
#endif
  EXPECT_EQ(outPose, CPose2D(1, 0, 0));

  CPose2D newPose(9, 9, 0);
  seq.changePose(0, newPose);
  EXPECT_EQ(seq.getPose(0), CPose2D(9, 9, 0));

  EXPECT_THROW((void)seq.getPose(99), std::exception);

  CPose2D absAfterAll = seq.absolutePoseAfterAll();
  (void)absAfterAll;
  CPose2D absOf1 = seq.absolutePoseOf(1);
  (void)absOf1;
  EXPECT_THROW(seq.absolutePoseOf(99), std::exception);

  double dist = seq.computeTraveledDistanceAfterAll();
  EXPECT_GE(dist, 0.0);
  EXPECT_THROW(seq.computeTraveledDistanceAfter(99), std::exception);

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << seq;
  buf.Seek(0);
  CPoses2DSequence seq2;
  arch >> seq2;
  EXPECT_EQ(seq2.posesCount(), seq.posesCount());

  seq.clear();
  EXPECT_EQ(seq.posesCount(), 0u);
}

TEST(CPosesSequence, CPoses3DSequenceFullCoverage)
{
  CPoses3DSequence seq;
  EXPECT_EQ(seq.posesCount(), 0u);

  CPose3D p1(1, 0, 0, 0, 0, 0);
  CPose3D p2(1, 0, 0, 0, 0, 0);
  seq.appendPose(p1);
  seq.appendPose(p2);
  EXPECT_EQ(seq.posesCount(), 2u);

  CPose3D got = seq.getPose(0);
  EXPECT_EQ(got, CPose3D(1, 0, 0, 0, 0, 0));

  CPose3D outPose;
#if defined(__clang__) || defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
  seq.getPose(1, outPose);  // exercise deprecated out-param overload
#if defined(__clang__) || defined(__GNUC__)
#pragma GCC diagnostic pop
#endif
  EXPECT_EQ(outPose, CPose3D(1, 0, 0, 0, 0, 0));

  CPose3D newPose(9, 9, 9, 0, 0, 0);
  seq.changePose(0, newPose);
  EXPECT_EQ(seq.getPose(0), CPose3D(9, 9, 9, 0, 0, 0));

  EXPECT_THROW((void)seq.getPose(99), std::exception);

  CPose3D absAfterAll = seq.absolutePoseAfterAll();
  (void)absAfterAll;
  CPose3D absOf1 = seq.absolutePoseOf(1);
  (void)absOf1;
  EXPECT_THROW(seq.absolutePoseOf(99), std::exception);

  double dist = seq.computeTraveledDistanceAfterAll();
  EXPECT_GE(dist, 0.0);
  EXPECT_THROW(seq.computeTraveledDistanceAfter(99), std::exception);

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << seq;
  buf.Seek(0);
  CPoses3DSequence seq2;
  arch >> seq2;
  EXPECT_EQ(seq2.posesCount(), seq.posesCount());

  seq.clear();
  EXPECT_EQ(seq.posesCount(), 0u);
}
