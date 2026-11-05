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

// Monte Carlo checks of CPoseRandomSampler.

#include <gtest/gtest.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/poses/CPosePDFParticles.h>
#include <mrpt/poses/CPosePDFSOG.h>
#include <mrpt/poses/CPoseRandomSampler.h>
#include <mrpt/random.h>

#include <Eigen/Dense>

using namespace mrpt;
using namespace mrpt::poses;
using namespace mrpt::math;

// Sample mean and covariance from a 2D Gaussian (with correlations and a
// heading close to pi) match the PDF.
TEST(CPoseRandomSamplerMath, Gaussian2D_SampleMoments)
{
  mrpt::random::getRandomGenerator().randomize(1234);
  Eigen::Matrix3d C;
  C << 0.04, 0.01, 0.002, 0.01, 0.09, -0.003, 0.002, -0.003, 0.01;
  const CPose2D mean(1.0, 2.0, M_PI - 0.05);
  CPoseRandomSampler s;
  s.setPosePDF(CPosePDFGaussian(mean, CMatrixDouble33(C)));

  const size_t N = 100000;
  Eigen::Matrix3d S = Eigen::Matrix3d::Zero();
  Eigen::Vector3d m = Eigen::Vector3d::Zero();
  for (size_t i = 0; i < N; i++)
  {
    CPose2D p;
    s.drawSample(p);
    Eigen::Vector3d d(p.x() - mean.x(), p.y() - mean.y(), wrapToPi(p.phi() - mean.phi()));
    EXPECT_LE(std::abs(p.phi()), M_PI + 1e-12);
    m += d / N;
    S += d * d.transpose() / N;
  }
  S -= m * m.transpose();
  EXPECT_NEAR(m.norm(), 0.0, 3e-3);
  EXPECT_NEAR((S - C).cwiseAbs().maxCoeff(), 0.0, 2e-3) << S;
}

// Same, for a 6D Gaussian.
TEST(CPoseRandomSamplerMath, Gaussian3D_SampleMoments)
{
  mrpt::random::getRandomGenerator().randomize(1234);
  CMatrixDouble66 r;
  mrpt::random::getRandomGenerator().drawGaussian1DMatrix(r, 0, 0.05);
  Eigen::Matrix<double, 6, 6> C = r.asEigen() * r.asEigen().transpose();
  C += 1e-5 * Eigen::Matrix<double, 6, 6>::Identity();
  const CPose3D mean(1, 2, 3, 0.3, -0.2, 0.1);
  CPoseRandomSampler s;
  s.setPosePDF(CPose3DPDFGaussian(mean, CMatrixDouble66(C)));

  const size_t N = 100000;
  Eigen::Matrix<double, 6, 6> S = Eigen::Matrix<double, 6, 6>::Zero();
  for (size_t i = 0; i < N; i++)
  {
    CPose3D p;
    s.drawSample(p);
    Eigen::Matrix<double, 6, 1> d;
    d << p.x() - mean.x(), p.y() - mean.y(), p.z() - mean.z(), wrapToPi(p.yaw() - mean.yaw()),
        wrapToPi(p.pitch() - mean.pitch()), wrapToPi(p.roll() - mean.roll());
    S += d * d.transpose() / N;
  }
  EXPECT_NEAR((S - C).cwiseAbs().maxCoeff(), 0.0, 0.03 * C.cwiseAbs().maxCoeff()) << S;
}

// The class documentation says any pose PDF is supported, and do_sample_2D()
// has a CPosePDFSOG branch.
TEST(CPoseRandomSamplerMath, SOG2D_Supported)
{
  mrpt::random::getRandomGenerator().randomize(1234);
  CPosePDFSOG sog(2);
  CMatrixDouble33 C;
  C.setIdentity();
  C *= 0.01;
  sog[0].mean = CPose2D(-5, 0, 0);
  sog[0].cov = C;
  sog[0].log_w = std::log(0.25);
  sog[1].mean = CPose2D(5, 0, 0);
  sog[1].cov = C;
  sog[1].log_w = std::log(0.75);

  CPoseRandomSampler s;
  bool ok = true;
  try
  {
    s.setPosePDF(sog);
  }
  catch (const std::exception&)
  {
    ok = false;
  }
  ASSERT_TRUE(ok);
  size_t n = 0;
  const size_t N = 10000;
  for (size_t i = 0; i < N; i++)
  {
    CPose2D p;
    s.drawSample(p);
    if (p.x() > 0)
    {
      n++;
    }
  }
  EXPECT_NEAR(static_cast<double>(n) / N, 0.75, 0.02);
}

// getSamplingMean2D() must describe the current PDF, also for particles.
TEST(CPoseRandomSamplerMath, SamplingMean_Particles)
{
  CMatrixDouble33 C;
  C.setIdentity();
  C *= 0.01;
  CPoseRandomSampler s;
  s.setPosePDF(CPosePDFGaussian(CPose2D(1, 1, 0), C));

  CPosePDFParticles parts(10);
  parts.resetDeterministic(TPose2D(5, 5, 0));
  s.setPosePDF(parts);
  CPose2D m;
  s.getSamplingMean2D(m);
  EXPECT_NEAR(m.x(), 5.0, 1e-9);
  EXPECT_NEAR(m.y(), 5.0, 1e-9);
}
