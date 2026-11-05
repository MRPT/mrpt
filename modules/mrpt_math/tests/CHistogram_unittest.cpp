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
#include <mrpt/math/CHistogram.h>
#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/math/ops_containers.h>
#include <mrpt/random/RandomGenerators.h>

#include <vector>

using mrpt::math::CHistogram;

TEST(CHistogram, constructorChecks)
{
  EXPECT_THROW(CHistogram(0.0, 100.0, 0), std::exception);
  EXPECT_THROW(CHistogram(100.0, 0.0, 10), std::exception);
  EXPECT_NO_THROW(CHistogram(0.0, 100.0, 10));
}

TEST(CHistogram, addAndCount)
{
  CHistogram h(0.0, 100.0, 10);

  h.add(86);
  h.add(7);
  h.add(45);

  EXPECT_EQ(h.getBinCount(0), 1U);
  EXPECT_EQ(h.getBinCount(4), 1U);
  EXPECT_EQ(h.getBinCount(8), 1U);
  EXPECT_EQ(h.getBinCount(9), 0U);
  EXPECT_NEAR(h.getBinRatio(0), 1.0 / 3.0, 1e-9);
  EXPECT_NEAR(h.getBinRatio(9), 0.0, 1e-9);

  // Values out of [min,max] are silently ignored:
  h.add(-1);
  h.add(101);
  EXPECT_NEAR(h.getBinRatio(0), 1.0 / 3.0, 1e-9);

  // The upper limit itself falls in the last bin:
  h.add(100);
  EXPECT_EQ(h.getBinCount(9), 1U);

  EXPECT_THROW((void)h.getBinCount(10), std::exception);
  EXPECT_THROW((void)h.getBinRatio(10), std::exception);

  h.clear();
  EXPECT_EQ(h.getBinCount(0), 0U);
  // With no elements added, the ratio is defined to be zero:
  EXPECT_EQ(h.getBinRatio(0), 0.0);
}

TEST(CHistogram, addContainers)
{
  CHistogram h(0.0, 10.0, 5);

  // Bins are [0,2), [2,4), ... so only 1.0 falls in the first one:
  const std::vector<double> v{1.0, 2.0, 3.0, 4.0, 5.0};
  h.add(v);
  EXPECT_EQ(h.getBinCount(0), 1U);

  mrpt::math::CVectorDouble cv(3);
  cv[0] = 1.0;
  cv[1] = 1.5;
  cv[2] = 9.0;
  h.add(cv);
  EXPECT_EQ(h.getBinCount(0), 3U);
  EXPECT_EQ(h.getBinCount(4), 1U);
}

TEST(CHistogram, getHistogram)
{
  CHistogram h(0.0, 10.0, 5);
  for (int i = 0; i < 10; i++) h.add(i + 0.5);

  std::vector<double> x, hits;
  h.getHistogram(x, hits);
  ASSERT_EQ(x.size(), 5U);
  ASSERT_EQ(hits.size(), 5U);
  for (const auto v : hits) EXPECT_NEAR(v, 2.0, 1e-9);

  std::vector<double> xn, hitsn;
  h.getHistogramNormalized(xn, hitsn);
  ASSERT_EQ(hitsn.size(), 5U);
  // The normalized histogram must integrate to 1:
  double integral = 0;
  const double binWidth = 10.0 / 5;
  for (const auto v : hitsn) integral += v * binWidth;
  EXPECT_NEAR(integral, 1.0, 1e-9);
}

TEST(CHistogram, createWithFixedWidth)
{
  const CHistogram h = CHistogram::createWithFixedWidth(0.0, 10.0, 2.0);
  std::vector<double> x, hits;
  h.getHistogram(x, hits);
  EXPECT_EQ(hits.size(), 5U);

  EXPECT_THROW((void)CHistogram::createWithFixedWidth(10.0, 0.0, 2.0), std::exception);
  EXPECT_THROW((void)CHistogram::createWithFixedWidth(0.0, 10.0, 0.0), std::exception);
}

TEST(CHistogram, getHistogramReturnsBinCenters)
{
  CHistogram h(0.0, 100.0, 10);
  std::vector<double> x;
  std::vector<double> hits;
  h.getHistogram(x, hits);
  ASSERT_EQ(x.size(), 10U);
  for (size_t i = 0; i < x.size(); i++)
  {
    EXPECT_NEAR(x[i], 5.0 + 10.0 * static_cast<double>(i), 1e-9) << "i=" << i;
  }
  // No samples: a normalized histogram of zeros, not NaN
  h.getHistogramNormalized(x, hits);
  ASSERT_EQ(x.size(), 10U);
  EXPECT_NEAR(x[0], 5.0, 1e-9);
  EXPECT_NEAR(x[9], 95.0, 1e-9);
  for (const auto v : hits)
  {
    EXPECT_EQ(v, 0.0);
  }
}

TEST(CHistogram, createWithFixedWidthHonorsWidth)
{
  auto h = CHistogram::createWithFixedWidth(0.0, 1.0, 0.3);
  std::vector<double> x;
  std::vector<double> hits;
  h.getHistogram(x, hits);
  ASSERT_EQ(x.size(), 4U);
  EXPECT_NEAR(x[0], 0.15, 1e-12);
  EXPECT_NEAR(x[3], 1.05, 1e-12);
  h.add(0.29);
  h.add(0.31);
  h.add(1.0);
  EXPECT_EQ(h.getBinCount(0), 1U);
  EXPECT_EQ(h.getBinCount(1), 1U);
  EXPECT_EQ(h.getBinCount(3), 1U);

  // A range that is a multiple of the width, up to round-off:
  auto h2 = CHistogram::createWithFixedWidth(0.0, 1.1, 0.1);
  h2.getHistogram(x, hits);
  EXPECT_EQ(x.size(), 11U);
}

TEST(CHistogram, histogramNormalizedIntegratesToOne)
{
  auto& rng = mrpt::random::getRandomGenerator();
  rng.randomize(1);
  std::vector<double> v(5000);
  for (auto& x : v)
  {
    x = rng.drawUniform(-2.0, 3.0);
  }
  std::vector<double> centers;
  const auto H = mrpt::math::histogram(v, -2.0, 3.0, 25, true, &centers);
  ASSERT_EQ(H.size(), 25U);
  ASSERT_EQ(centers.size(), 25U);
  EXPECT_NEAR(centers[0], -1.9, 1e-12);
  double integral = 0;
  for (const double hv : H)
  {
    integral += hv * (5.0 / 25.0);
  }
  EXPECT_NEAR(integral, 1.0, 1e-12);
  // Uniform density 1/5 on each bin, within Monte Carlo noise
  for (const double hv : H)
  {
    EXPECT_NEAR(hv, 0.2, 0.05);
  }
}
