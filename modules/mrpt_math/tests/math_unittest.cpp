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

#include <cmath>
#include <sstream>
#include <string>
#include <vector>

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

namespace
{
// Parses the two numeric lists in "plot([ x0,x1,...],[ y0,y1,...],'style');"
void parseMatlabPlot(const std::string& s, std::vector<double>& xs, std::vector<double>& ys)
{
  xs.clear();
  ys.clear();
  const auto p0 = s.find('[');
  const auto p1 = s.find(']', p0);
  const auto p2 = s.find('[', p1);
  const auto p3 = s.find(']', p2);
  ASSERT_NE(p3, std::string::npos);
  auto parseList = [](const std::string& str, std::vector<double>& out)
  {
    std::stringstream ss(str);
    std::string tok;
    while (std::getline(ss, tok, ','))
    {
      out.push_back(std::stod(tok));
    }
  };
  parseList(s.substr(p0 + 1, p1 - p0 - 1), xs);
  parseList(s.substr(p2 + 1, p3 - p2 - 1), ys);
}
}  // namespace

TEST(MathMisc, AverageLogLikelihoodWeightedStableWhenPeaksDiffer)
{
  CVectorDouble lw(2);
  CVectorDouble ll(2);
  lw[0] = 0;
  lw[1] = -1000;
  ll[0] = -1000;
  ll[1] = 0;
  // log((e^-1000 + e^-1000) / (1 + e^-1000)) = log(2) - 1000
  double r = 0;
  EXPECT_NO_THROW(r = mrpt::math::averageLogLikelihood(lw, ll));
  EXPECT_NEAR(r, std::log(2.0) - 1000.0, 1e-9);
}

TEST(MathMisc, AverageLogLikelihoodWeightedMatchesDirectFormula)
{
  const double lwv[] = {-1.0, 0.5, -3.0, 2.0};
  const double llv[] = {-2.0, -0.1, 1.0, -4.0};
  CVectorDouble lw(4);
  CVectorDouble ll(4);
  double num = 0;
  double den = 0;
  for (int i = 0; i < 4; i++)
  {
    lw[i] = lwv[i];
    ll[i] = llv[i];
    num += std::exp(lwv[i] + llv[i]);
    den += std::exp(lwv[i]);
  }
  const double ref = std::log(num / den);
  EXPECT_NEAR(mrpt::math::averageLogLikelihood(lw, ll), ref, 1e-12);

  // Invariant to the scale of the weights, and stable for large offsets:
  CVectorDouble lw2 = lw;
  CVectorDouble ll2 = ll;
  for (int i = 0; i < 4; i++)
  {
    lw2[i] += 5000.0;
    ll2[i] -= 3000.0;
  }
  EXPECT_NEAR(mrpt::math::averageLogLikelihood(lw2, ll2), ref - 3000.0, 1e-9);
}

TEST(MathMisc, AverageLogLikelihoodUnweightedStableAndExact)
{
  CVectorDouble ll(3);
  ll[0] = -5000.0;
  ll[1] = -5001.0;
  ll[2] = -5002.0;
  const double ref = -5000.0 + std::log((1.0 + std::exp(-1.0) + std::exp(-2.0)) / 3.0);
  EXPECT_NEAR(mrpt::math::averageLogLikelihood(ll), ref, 1e-9);
}

TEST(MathMisc, AverageWrap2PiIsCircularMean)
{
  CVectorDouble a(2);
  a[0] = -1.4;
  a[1] = 2.9;
  // Short arc midpoint:
  EXPECT_NEAR(mrpt::math::averageWrap2Pi(a), -1.4 - (2 * M_PI - 4.3) / 2, 1e-12);

  CVectorDouble b(2);
  b[0] = 2.0;
  b[1] = -2.0;
  EXPECT_NEAR(std::abs(mrpt::math::averageWrap2Pi(b)), M_PI, 1e-12);

  CVectorDouble c(3);
  c[0] = 0.1;
  c[1] = 0.2;
  c[2] = 0.3;
  EXPECT_NEAR(mrpt::math::averageWrap2Pi(c), 0.2, 1e-12);
}

TEST(MathMisc, MatlabPlotCovariance2DPointsLieOnTheEllipse)
{
  CMatrixDouble cov(2, 2);
  cov(0, 0) = 1.0;
  cov(1, 1) = 4.0;
  cov(0, 1) = 0.0;
  cov(1, 0) = 0.0;
  CVectorDouble mean(2);
  mean[0] = 10.0;
  mean[1] = 20.0;
  const auto s = mrpt::math::MATLAB_plotCovariance2D(cov, mean, 2.0f, "b", 13);
  std::vector<double> xs;
  std::vector<double> ys;
  parseMatlabPlot(s, xs, ys);
  ASSERT_EQ(xs.size(), 13U);
  ASSERT_EQ(ys.size(), 13U);
  for (size_t i = 0; i < xs.size(); i++)
  {
    // Mahalanobis distance of each point must be stdCount=2:
    const double d2 = mrpt::square(xs[i] - 10.0) / 1.0 + mrpt::square(ys[i] - 20.0) / 4.0;
    EXPECT_NEAR(d2, 4.0, 1e-3) << "i=" << i;
  }
}

TEST(MathMisc, MedianFilterBoundaries)
{
  {
    const std::vector<double> in{5, 5, 5, 5, 5};
    std::vector<double> out;
    mrpt::math::medianFilter(in, out, 3);
    ASSERT_EQ(out.size(), in.size());
    for (size_t i = 0; i < out.size(); i++)
    {
      EXPECT_DOUBLE_EQ(out[i], 5.0) << "i=" << i;
    }
  }
  {
    const std::vector<double> in{1, 5, 2, 8, 3};
    std::vector<double> out;
    mrpt::math::medianFilter(in, out, 3);
    ASSERT_EQ(out.size(), in.size());
    // Truncated windows at both ends: mean of the two middle values
    EXPECT_DOUBLE_EQ(out[0], 3.0);
    EXPECT_DOUBLE_EQ(out[1], 2.0);
    EXPECT_DOUBLE_EQ(out[2], 5.0);
    EXPECT_DOUBLE_EQ(out[3], 3.0);
    EXPECT_DOUBLE_EQ(out[4], 5.5);
  }
}
