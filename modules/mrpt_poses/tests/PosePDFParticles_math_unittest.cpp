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
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/poses/CPose3DPDFParticles.h>
#include <mrpt/poses/CPosePDFGaussian.h>
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

namespace
{
// Product of two Gaussians, with the second covariance inflated by the
// kernel of a Gaussian kernel density estimate with Silverman's bandwidth.
template <int D>
void expectedKdeFusion(
    const Eigen::Matrix<double, D, 1>& m1,
    const Eigen::Matrix<double, D, D>& C1,
    const Eigen::Matrix<double, D, 1>& m2,
    const Eigen::Matrix<double, D, D>& C2,
    size_t N2,
    Eigen::Matrix<double, D, 1>& m,
    Eigen::Matrix<double, D, D>& C)
{
  const double h2 = std::pow(4.0 / ((D + 2) * static_cast<double>(N2)), 2.0 / (D + 4));
  const Eigen::Matrix<double, D, D> C2k = C2 * (1 + h2);
  C = (C1.inverse() + C2k.inverse()).inverse();
  m = C * (C1.inverse() * m1 + C2k.inverse() * m2);
}
}  // namespace

// Regression: the kernel density estimate of the second particle PDF used an
// isotropic bandwidth equal to its standard deviation, which inflated the
// fused covariances (up to 2x along the most certain directions).
TEST(CPosePDFParticlesMath, BayesianFusion_KernelDensity)
{
  mrpt::random::getRandomGenerator().randomize(1234);
  Eigen::Matrix3d C1;
  C1 << 1.0, 0.02, 0.0, 0.02, 0.01, 0.0, 0.0, 0.0, 0.04;
  Eigen::Matrix3d C2;
  C2 << 0.5, -0.01, 0.01, -0.01, 0.02, 0.0, 0.01, 0.0, 0.03;
  const Eigen::Vector3d m1(0.0, 0.0, 0.1);
  const Eigen::Vector3d m2(0.5, 0.05, 0.2);
  const size_t N = 3000;
  CPosePDFParticles p1(N);
  CPosePDFParticles p2(N);
  p1.copyFrom(CPosePDFGaussian(CPose2D(m1[0], m1[1], m1[2]), CMatrixDouble33(C1)));
  p2.copyFrom(CPosePDFGaussian(CPose2D(m2[0], m2[1], m2[2]), CMatrixDouble33(C2)));
  CPosePDFParticles fused;
  fused.bayesianFusion(p1, p2);
  const auto [C, mean] = fused.getCovarianceAndMean();

  Eigen::Vector3d em;
  Eigen::Matrix3d eC;
  expectedKdeFusion<3>(m1, C1, m2, C2, N, em, eC);
  EXPECT_NEAR(mean.x(), em[0], 0.05 * std::sqrt(eC(0, 0)) + 0.03);
  EXPECT_NEAR(mean.y(), em[1], 0.05 * std::sqrt(eC(1, 1)) + 0.003);
  EXPECT_NEAR(mean.phi(), em[2], 0.05 * std::sqrt(eC(2, 2)) + 0.006);
  for (int i = 0; i < 3; i++)
  {
    EXPECT_NEAR(C(i, i), eC(i, i), 0.08 * eC(i, i)) << "i=" << i;
  }
}

// Same check for 3D poses (6D kernel).
TEST(CPose3DPDFParticlesMath, BayesianFusion_KernelDensity)
{
  mrpt::random::getRandomGenerator().randomize(1234);
  Eigen::Matrix<double, 6, 6> C1 = Eigen::Matrix<double, 6, 6>::Zero();
  Eigen::Matrix<double, 6, 6> C2 = Eigen::Matrix<double, 6, 6>::Zero();
  const double v1[6] = {0.25, 0.25, 0.0025, 0.01, 0.01, 0.01};
  const double v2[6] = {0.16, 0.36, 0.0036, 0.02, 0.005, 0.01};
  for (int i = 0; i < 6; i++)
  {
    C1(i, i) = v1[i];
    C2(i, i) = v2[i];
  }
  Eigen::Matrix<double, 6, 1> m1;
  Eigen::Matrix<double, 6, 1> m2;
  m1 << 0, 0, 0, 0.1, 0, 0;
  m2 << 0.3, -0.2, 0.02, 0.15, -0.05, 0.02;
  const size_t N = 2000;
  CPose3DPDFParticles p1(N);
  CPose3DPDFParticles p2(N);
  p1.copyFrom(
      CPose3DPDFGaussian(CPose3D(m1[0], m1[1], m1[2], m1[3], m1[4], m1[5]), CMatrixDouble66(C1)));
  p2.copyFrom(
      CPose3DPDFGaussian(CPose3D(m2[0], m2[1], m2[2], m2[3], m2[4], m2[5]), CMatrixDouble66(C2)));
  CPose3DPDFParticles fused;
  fused.bayesianFusion(p1, p2);
  const auto [C, mean] = fused.getCovarianceAndMean();

  Eigen::Matrix<double, 6, 1> em;
  Eigen::Matrix<double, 6, 6> eC;
  expectedKdeFusion<6>(m1, C1, m2, C2, N, em, eC);
  const double meanv[6] = {mean.x(), mean.y(), mean.z(), mean.yaw(), mean.pitch(), mean.roll()};
  for (int i = 0; i < 6; i++)
  {
    EXPECT_NEAR(meanv[i], em[i], 0.1 * std::sqrt(eC(i, i))) << "i=" << i;
    EXPECT_NEAR(C(i, i), eC(i, i), 0.1 * eC(i, i)) << "i=" << i;
  }
}
