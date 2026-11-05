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

// Math checks for particle-based PDFs: weighted moments with angular wrap,
// and sampling from the particle set.

#include <gtest/gtest.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/poses/CPointPDFParticles.h>
#include <mrpt/poses/CPose3DPDFParticles.h>
#include <mrpt/poses/CPosePDFGaussianInf.h>
#include <mrpt/poses/CPosePDFParticles.h>
#include <mrpt/random.h>

#include <Eigen/Dense>

using namespace mrpt;
using namespace mrpt::poses;
using namespace mrpt::math;

// Weighted covariance of particles whose heading is linearly correlated with
// x: the heading errors must keep their sign.
TEST(CPosePDFParticlesMath, Covariance_CrossTermsSign)
{
  const size_t N = 101;
  CPosePDFParticles p(N);
  double varX = 0;
  for (size_t i = 0; i < N; i++)
  {
    const double x = -1.0 + 2.0 * static_cast<double>(i) / (N - 1);
    p.m_particles[i].d = TPose2D(x, 0.5 * x, 0.5 + 0.1 * x);
    p.m_particles[i].log_w = 0;
    varX += x * x / N;
  }
  const auto [C, m] = p.getCovarianceAndMean();
  EXPECT_NEAR(m.phi(), 0.5, 1e-6);
  EXPECT_NEAR(C(0, 0), varX, 1e-9);
  EXPECT_NEAR(C(0, 1), 0.5 * varX, 1e-9);
  EXPECT_NEAR(C(2, 2), 0.01 * varX, 1e-6);
  EXPECT_NEAR(C(0, 2), 0.1 * varX, 1e-6);
  EXPECT_NEAR(C(1, 2), 0.05 * varX, 1e-6);
}

// Same check, 6D version.
TEST(CPose3DPDFParticlesMath, Covariance_CrossTermsSign)
{
  const size_t N = 101;
  CPose3DPDFParticles p(N);
  double varX = 0;
  for (size_t i = 0; i < N; i++)
  {
    const double x = -1.0 + 2.0 * static_cast<double>(i) / (N - 1);
    p.m_particles[i].d = TPose3D(x, 0, 0, 0.3 + 0.1 * x, -0.05 * x, 0.02 * x);
    p.m_particles[i].log_w = 0;
    varX += x * x / N;
  }
  const auto [C, m] = p.getCovarianceAndMean();
  EXPECT_NEAR(C(0, 0), varX, 1e-9);
  EXPECT_NEAR(C(0, 3), 0.1 * varX, 1e-5);
  EXPECT_NEAR(C(0, 4), -0.05 * varX, 1e-5);
  EXPECT_NEAR(C(0, 5), 0.02 * varX, 1e-5);
  EXPECT_NEAR(C(3, 4), -0.005 * varX, 1e-5);
}

// Mean and heading variance of particles straddling +-pi.
TEST(CPosePDFParticlesMath, MeanAndVariance_AcrossPi)
{
  CPosePDFParticles p(2);
  p.m_particles[0].d = TPose2D(1, 0, M_PI - 0.1);
  p.m_particles[1].d = TPose2D(1, 0, -M_PI + 0.1);
  p.m_particles[0].log_w = 0;
  p.m_particles[1].log_w = 0;
  const auto [C, m] = p.getCovarianceAndMean();
  EXPECT_NEAR(std::abs(mrpt::math::wrapToPi(m.phi() - M_PI)), 0.0, 1e-9);
  EXPECT_NEAR(C(2, 2), 0.01, 1e-9);
}

// The weighted moments must not depend on a common offset of the
// log-weights, even if exp(log_w) underflows.
TEST(CPosePDFParticlesMath, Moments_InvariantToLogWeightOffset)
{
  CPosePDFParticles p(3);
  p.m_particles[0].d = TPose2D(0, 0, 0);
  p.m_particles[1].d = TPose2D(1, 0, 0);
  p.m_particles[2].d = TPose2D(2, 0, 0);
  for (size_t i = 0; i < 3; i++)
  {
    p.m_particles[i].log_w = -1000.0 + static_cast<double>(i);
  }
  CPose2D mean(UNINITIALIZED_POSE);
  bool threw = false;
  try
  {
    p.getMean(mean);
  }
  catch (const std::exception&)
  {
    threw = true;
  }
  EXPECT_FALSE(threw);
  const double w0 = 1.0;
  const double w1 = std::exp(1.0);
  const double w2 = std::exp(2.0);
  if (!threw)
  {
    EXPECT_NEAR(mean.x(), (w1 + 2 * w2) / (w0 + w1 + w2), 1e-9);
  }
}

namespace
{
// Fraction of samples whose x coordinate is positive.
template <class PDF, class SAMPLE>
double fractionPositiveX(const PDF& pdf, size_t N)
{
  size_t n = 0;
  for (size_t i = 0; i < N; i++)
  {
    SAMPLE s;
    pdf.drawSingleSample(s);
    if (s.x() > 0)
    {
      n++;
    }
  }
  return static_cast<double>(n) / static_cast<double>(N);
}
}  // namespace

// Two equally weighted particles must be drawn with equal frequency, for the
// weights as left by the constructor and by normalizeWeights() (max log_w=0).
TEST(PosePDFParticlesMath, DrawSingleSample_UsesNormalizedWeights)
{
  mrpt::random::getRandomGenerator().randomize(1234);
  const size_t N = 4000;

  CPosePDFParticles p2(2);
  p2.m_particles[0].d = TPose2D(-1, 0, 0);
  p2.m_particles[1].d = TPose2D(1, 0, 0);
  EXPECT_NEAR((fractionPositiveX<CPosePDFParticles, CPose2D>(p2, N)), 0.5, 0.05);

  CPose3DPDFParticles p3(2);
  p3.m_particles[0].d = TPose3D(-1, 0, 0, 0, 0, 0);
  p3.m_particles[1].d = TPose3D(1, 0, 0, 0, 0, 0);
  EXPECT_NEAR((fractionPositiveX<CPose3DPDFParticles, CPose3D>(p3, N)), 0.5, 0.05);

  CPointPDFParticles pp(2);
  pp.m_particles[0].d->x = -1;
  pp.m_particles[1].d->x = 1;
  EXPECT_NEAR((fractionPositiveX<CPointPDFParticles, CPoint3D>(pp, N)), 0.5, 0.05);
}

// copyFrom() of any other PDF type must either convert it or throw.
TEST(CPosePDFParticlesMath, CopyFrom_GaussianInf)
{
  mrpt::random::getRandomGenerator().randomize(1234);
  CMatrixDouble33 I;
  I.setIdentity();
  I *= 1e4;
  const CPosePDFGaussianInf g(CPose2D(5, 5, 0), I);
  CPosePDFParticles p(100);
  bool threw = false;
  try
  {
    p.copyFrom(g);
  }
  catch (const std::exception&)
  {
    threw = true;
  }
  if (!threw)
  {
    EXPECT_NEAR(p.getMeanVal().x(), 5.0, 0.05);
  }
}

// Weighted mean and covariance of a 3D point particle set.
TEST(CPointPDFParticlesMath, CovarianceAndMean_ClosedForm)
{
  CPointPDFParticles p(3);
  const double w[3] = {0.2, 0.3, 0.5};
  const Eigen::Vector3d pts[3] = {
      { 0, 0, 0},
      { 1, 2, 0},
      {-1, 1, 3}
  };
  Eigen::Vector3d mu = Eigen::Vector3d::Zero();
  for (int i = 0; i < 3; i++)
  {
    p.m_particles[i].d->x = static_cast<float>(pts[i].x());
    p.m_particles[i].d->y = static_cast<float>(pts[i].y());
    p.m_particles[i].d->z = static_cast<float>(pts[i].z());
    p.m_particles[i].log_w = std::log(w[i]) + 7.0;
    mu += w[i] * pts[i];
  }
  Eigen::Matrix3d C = Eigen::Matrix3d::Zero();
  for (int i = 0; i < 3; i++)
  {
    C += w[i] * (pts[i] - mu) * (pts[i] - mu).transpose();
  }
  const auto [Cout, m] = p.getCovarianceAndMean();
  EXPECT_NEAR((Eigen::Vector3d(m.x(), m.y(), m.z()) - mu).norm(), 0.0, 1e-6);
  EXPECT_NEAR((Cout.asEigen() - C).cwiseAbs().maxCoeff(), 0.0, 1e-6);
}
