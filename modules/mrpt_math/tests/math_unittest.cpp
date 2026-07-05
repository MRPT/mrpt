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
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/math/data_utils.h>
#include <mrpt/math/distributions.h>
#include <mrpt/math/interp_fit.h>
#include <mrpt/math/utils.h>

#include <sstream>

using namespace mrpt::math;

TEST(MathMisc, Factorial)
{
  EXPECT_NEAR(mrpt::math::factorial(0), 1.0, 1e-9);
  EXPECT_NEAR(mrpt::math::factorial(1), 1.0, 1e-9);
  EXPECT_NEAR(mrpt::math::factorial(5), 120.0, 1e-6);
}

TEST(MathMisc, Factorial64)
{
  EXPECT_EQ(mrpt::math::factorial64(0), 1u);
  EXPECT_EQ(mrpt::math::factorial64(1), 1u);
  EXPECT_EQ(mrpt::math::factorial64(10), 3628800u);
}

TEST(MathMisc, NormalQuantileTailRegion)
{
  // q = min(p,1-p) <= 0.02425 exercises the "tail region" rational
  // approximation branch, and p>0.5 exercises the final sign flip.
  const double lowQ = mrpt::math::normalQuantile(0.001);
  const double highQ = mrpt::math::normalQuantile(0.999);
  EXPECT_LT(lowQ, -2.5);
  EXPECT_GT(highQ, 2.5);
  EXPECT_NEAR(lowQ, -highQ, 1e-6);
}

TEST(MathMisc, Chi2invTailRegion)
{
  EXPECT_GT(mrpt::math::chi2inv(0.001, 3), 0.0);
  EXPECT_GT(mrpt::math::chi2inv(0.999, 3), 0.0);
}

TEST(MathMisc, LoadVectorInt)
{
  std::istringstream ss("1 2 3\n4 5\n");
  std::vector<int> v;
  ASSERT_TRUE(mrpt::math::loadVector(ss, v));
  ASSERT_EQ(v.size(), 3u);
  EXPECT_EQ(v[0], 1);

  ASSERT_TRUE(mrpt::math::loadVector(ss, v));
  ASSERT_EQ(v.size(), 2u);
  EXPECT_EQ(v[1], 5);

  // No more lines:
  EXPECT_FALSE(mrpt::math::loadVector(ss, v));
}

TEST(MathMisc, LoadVectorDouble)
{
  std::istringstream ss("1.5 2.5 3.5\n");
  std::vector<double> v;
  ASSERT_TRUE(mrpt::math::loadVector(ss, v));
  ASSERT_EQ(v.size(), 3u);
  EXPECT_NEAR(v[2], 3.5, 1e-9);
}

TEST(MathMisc, AverageLogLikelihoodWeighted)
{
  CVectorDouble logWeights(3);
  logWeights[0] = 0.0;
  logWeights[1] = 0.0;
  logWeights[2] = 0.0;
  CVectorDouble logLikelihoods(3);
  logLikelihoods[0] = -1.0;
  logLikelihoods[1] = -1.0;
  logLikelihoods[2] = -1.0;

  const double r = mrpt::math::averageLogLikelihood(logWeights, logLikelihoods);
  EXPECT_NEAR(r, -1.0, 1e-6);
}

TEST(MathMisc, AverageLogLikelihoodWeightedEmptyThrows)
{
  CVectorDouble logWeights;
  CVectorDouble logLikelihoods;
  EXPECT_THROW(mrpt::math::averageLogLikelihood(logWeights, logLikelihoods), std::exception);
}

TEST(MathMisc, AverageLogLikelihoodUnweighted)
{
  CVectorDouble logLikelihoods(2);
  logLikelihoods[0] = -2.0;
  logLikelihoods[1] = -2.0;
  const double r = mrpt::math::averageLogLikelihood(logLikelihoods);
  EXPECT_NEAR(r, -2.0, 1e-6);
}

TEST(MathMisc, AverageLogLikelihoodUnweightedEmptyThrows)
{
  CVectorDouble logLikelihoods;
  EXPECT_THROW(mrpt::math::averageLogLikelihood(logLikelihoods), std::exception);
}

TEST(MathMisc, AverageWrap2PiEmpty)
{
  CVectorDouble angles;
  EXPECT_NEAR(mrpt::math::averageWrap2Pi(angles), 0.0, 1e-9);
}

TEST(MathMisc, AverageWrap2PiMixedHalves)
{
  // Angles near +/-pi, split between the "left" (>pi/2) and "right" halves.
  CVectorDouble angles(4);
  angles[0] = 3.0;
  angles[1] = -3.0;
  angles[2] = 0.1;
  angles[3] = -0.1;
  const double r = mrpt::math::averageWrap2Pi(angles);
  EXPECT_GE(r, -M_PI);
  EXPECT_LE(r, M_PI);
}

TEST(MathMisc, MatlabPlotCovariance2DFloatAndDouble)
{
  CMatrixDouble cov(2, 2);
  cov(0, 0) = 1.0;
  cov(1, 1) = 2.0;
  cov(0, 1) = cov(1, 0) = 0.0;
  CVectorDouble mean(2);
  mean[0] = 1.0;
  mean[1] = 2.0;

  const auto s = mrpt::math::MATLAB_plotCovariance2D(cov, mean, 2.0f, "b-", 10);
  EXPECT_FALSE(s.empty());

  CMatrixFloat covf(2, 2);
  covf(0, 0) = 1.0f;
  covf(1, 1) = 2.0f;
  CVectorFloat meanf(2);
  meanf[0] = 1.0f;
  meanf[1] = 2.0f;
  const auto sf = mrpt::math::MATLAB_plotCovariance2D(covf, meanf, 2.0f, "b-", 10);
  EXPECT_FALSE(sf.empty());
}

TEST(MathMisc, Interpolate2Points)
{
  EXPECT_NEAR(mrpt::math::interpolate2points(0.5, 0.0, 0.0, 1.0, 10.0), 5.0, 1e-9);
}

TEST(MathMisc, Interpolate2PointsWrap2Pi)
{
  const double r = mrpt::math::interpolate2points(0.5, 0.0, 3.0, 1.0, -3.0, true);
  EXPECT_GE(r, -M_PI);
  EXPECT_LE(r, M_PI);
}

TEST(MathMisc, Interpolate2PointsThrowsOnEqualX)
{
  EXPECT_THROW(mrpt::math::interpolate2points(0.5, 1.0, 0.0, 1.0, 10.0), std::exception);
}

TEST(MathMisc, MedianFilterOddWindow)
{
  const std::vector<double> in{1, 5, 2, 8, 3};
  std::vector<double> out;
  mrpt::math::medianFilter(in, out, 3);
  ASSERT_EQ(out.size(), in.size());
}

TEST(MathMisc, MedianFilterEvenWindowBecomesOdd)
{
  const std::vector<double> in{1, 5, 2, 8, 3, 9};
  std::vector<double> out;
  // An even window size (4) is internally bumped to an odd size (5).
  mrpt::math::medianFilter(in, out, 4);
  ASSERT_EQ(out.size(), in.size());
}

TEST(MathMisc, Chi2CDF)
{
  EXPECT_GT(mrpt::math::chi2CDF(3, 2.0), 0.0);
  EXPECT_LT(mrpt::math::chi2CDF(3, 2.0), 1.0);
}

TEST(MathMisc, NoncentralChi2CDF)
{
  const double r = mrpt::math::noncentralChi2CDF(3, 1.0, 2.0);
  EXPECT_GT(r, 0.0);
  EXPECT_LT(r, 1.0);
}

TEST(MathMisc, NoncentralChi2PDF_CDF_ZeroDegreesOfFreedom)
{
  // degreesOfFreedom==0 exercises a distinct branch in the internal series
  // expansion.
  const auto r = mrpt::math::noncentralChi2PDF_CDF(0, 2.0, 1.0);
  EXPECT_GE(r.first, 0.0);
  EXPECT_GE(r.second, 0.0);
  EXPECT_LE(r.second, 1.0);
}
