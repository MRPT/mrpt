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

// Multivariate Gaussian sampling of mrpt_random, with MRPT matrices.

#include <gtest/gtest.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/random/RandomGenerators.h>

#include <Eigen/Dense>
#include <cmath>
#include <vector>

TEST(drawGaussianMultivariate, SampleCovarianceWithMean)
{
  mrpt::random::CRandomGenerator rng(2024);
  mrpt::math::CMatrixDouble33 C;
  C.asEigen() << 4.0, 2.0, 1.0, 2.0, 3.0, 0.5, 1.0, 0.5, 1.0;
  mrpt::math::CVectorDouble mean(3);
  mean[0] = 1.0;
  mean[1] = -2.0;
  mean[2] = 3.0;

  const int N = 100000;
  Eigen::Vector3d s = Eigen::Vector3d::Zero();
  Eigen::Matrix3d s2 = Eigen::Matrix3d::Zero();
  mrpt::math::CVectorDouble x;
  for (int i = 0; i < N; i++)
  {
    rng.drawGaussianMultivariate(x, C, &mean);
    ASSERT_EQ(x.size(), 3);
    const Eigen::Vector3d e(x[0], x[1], x[2]);
    s += e;
    s2 += e * e.transpose();
  }
  const Eigen::Vector3d m = s / N;
  const Eigen::Matrix3d cov = s2 / N - m * m.transpose();
  for (int i = 0; i < 3; i++)
  {
    EXPECT_NEAR(m[i], mean[i], 0.04);
    for (int j = 0; j < 3; j++)
    {
      EXPECT_NEAR(cov(i, j), C(i, j), 0.12) << "i=" << i << " j=" << j;
    }
  }
}

TEST(drawGaussianMultivariate, ManySampleCovariance)
{
  mrpt::random::CRandomGenerator rng(99);
  mrpt::math::CMatrixDouble C(2, 2);
  C(0, 0) = 1.0;
  C(0, 1) = -0.8;
  C(1, 0) = -0.8;
  C(1, 1) = 2.0;
  std::vector<double> mean = {5.0, -5.0};
  std::vector<std::vector<double>> samples;
  const size_t N = 100000;
  rng.drawGaussianMultivariateMany(samples, N, C, &mean);
  ASSERT_EQ(samples.size(), N);
  double m0 = 0;
  double m1 = 0;
  for (const auto& v : samples)
  {
    ASSERT_EQ(v.size(), 2U);
    m0 += v[0];
    m1 += v[1];
  }
  m0 /= N;
  m1 /= N;
  double c00 = 0;
  double c01 = 0;
  double c11 = 0;
  for (const auto& v : samples)
  {
    c00 += (v[0] - m0) * (v[0] - m0);
    c01 += (v[0] - m0) * (v[1] - m1);
    c11 += (v[1] - m1) * (v[1] - m1);
  }
  EXPECT_NEAR(m0, 5.0, 0.02);
  EXPECT_NEAR(m1, -5.0, 0.03);
  EXPECT_NEAR(c00 / N, 1.0, 0.03);
  EXPECT_NEAR(c01 / N, -0.8, 0.03);
  EXPECT_NEAR(c11 / N, 2.0, 0.05);
}

TEST(drawGaussianMultivariate, SemidefiniteCovariance)
{
  mrpt::random::CRandomGenerator rng(1);
  for (int t = 0; t < 50; t++)
  {
    Eigen::Vector3d v;
    for (int i = 0; i < 3; i++)
    {
      v[i] = rng.drawGaussian1D_normalized();
    }
    mrpt::math::CMatrixDouble33 C;
    C.asEigen() = v * v.transpose();  // rank 1
    mrpt::math::CVectorDouble x;
    rng.drawGaussianMultivariate(x, C);
    const Eigen::Vector3d e(x[0], x[1], x[2]);
    ASSERT_TRUE(e.allFinite()) << "t=" << t;
    // Samples must lie on the line spanned by v:
    EXPECT_NEAR(e.cross(v).norm(), 0.0, 1e-6 * (1.0 + e.norm() * v.norm()));

    std::vector<std::vector<double>> many;
    rng.drawGaussianMultivariateMany(many, 3, C);
    for (const auto& s : many)
    {
      EXPECT_TRUE(std::isfinite(s[0]) && std::isfinite(s[1]) && std::isfinite(s[2]));
    }
  }
}

TEST(drawGaussianMultivariate, StdVectorOutputAndGlobalHelpers)
{
  mrpt::random::getRandomGenerator().randomize(5);
  mrpt::math::CMatrixDouble C(2, 2);
  C.setIdentity();
  std::vector<double> x;
  mrpt::random::getRandomGenerator().drawGaussianMultivariate(x, C);
  EXPECT_EQ(x.size(), 2U);
  mrpt::random::randomNormalMultiDimensional(C, x);
  EXPECT_EQ(x.size(), 2U);
  std::vector<std::vector<double>> r;
  mrpt::random::randomNormalMultiDimensionalMany(C, 10, r);
  EXPECT_EQ(r.size(), 10U);

  mrpt::math::CMatrixDouble nonSquare(2, 3);
  nonSquare.setZero();
  EXPECT_THROW(
      mrpt::random::getRandomGenerator().drawGaussianMultivariate(x, nonSquare), std::exception);
}

TEST(drawGaussianMultivariate, drawDefinitePositiveMatrixIsPositiveDefinite)
{
  mrpt::random::CRandomGenerator rng(3);
  for (int t = 0; t < 10; t++)
  {
    const auto C = rng.drawDefinitePositiveMatrix<mrpt::math::CMatrixDouble>(4, 2.0, 1e-3);
    EXPECT_TRUE(C.asEigen().isApprox(C.asEigen().transpose()));
    Eigen::LLT<Eigen::MatrixXd> llt(C.asEigen());
    EXPECT_EQ(llt.info(), Eigen::Success);
  }
}
