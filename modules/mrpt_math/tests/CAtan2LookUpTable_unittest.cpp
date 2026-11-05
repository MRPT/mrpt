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
#include <mrpt/core/bits_math.h>  // DEG2RAD
#include <mrpt/math/CAtan2LookUpTable.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/random.h>

#include <cmath>

template <class LUT_CLASS>
void atan2_lut_test(
    const LUT_CLASS& atan2lut,
    const double SIZE,
    const double max_deg_errors,
    const double skip_area)
{
  for (int i = 0; i < 1000; i++)
  {
    const double x = mrpt::random::getRandomGenerator().drawUniform(-.5 * SIZE, .5 * SIZE);
    const double y = mrpt::random::getRandomGenerator().drawUniform(-.5 * SIZE, .5 * SIZE);

    // Avoid the central part, where accuracy is worse
    if (std::abs(x) < skip_area || std::abs(y) < skip_area) continue;

    const double atan2_good = ::atan2(y, x);
    double atan2_lut = .0;
    bool atan2_lut_valid = atan2lut.atan2(y, x, atan2_lut);

    EXPECT_TRUE(atan2_lut_valid);
    EXPECT_LT(std::abs(atan2_good - atan2_lut), mrpt::DEG2RAD(max_deg_errors))
        << "(x,y): (" << x << " , " << y << ")"
        << "\natan2_good:" << atan2_good << " atan2_lut:" << atan2_lut << "\n";
  }
}

TEST(CAtan2LookUpTable, ValidValidTest)
{
  const double SIZE = 20.0;
  const double RES = 0.10;
  mrpt::math::CAtan2LookUpTable atan2lut(-.5 * SIZE, .5 * SIZE, -.5 * SIZE, .5 * SIZE, RES);
  atan2_lut_test(atan2lut, SIZE, 1.5 /*max error*/, 2.0 /* skip zone */);
}

TEST(CAtan2LookUpTable, MultiResTest)
{
  const double SIZE = 20.0;
  mrpt::math::CAtan2LookUpTableMultiRes atan2lut;

  std::map<double, double> res2extension;
  res2extension[0.001] = 0.8;  // 0.1 cm resolution
  res2extension[0.01] = 2.0;   // 1.0 cm resolution
  res2extension[0.02] = 5.0;   // 2.0 cm resolution
  res2extension[0.05] = 11.0;  // 5.0 cm resolution
  atan2lut.resize(res2extension);

  atan2_lut_test(atan2lut, SIZE, 0.5 /*max error*/, 0.4 /* skip zone */);
}

// The grid limits must be those requested, for any resolution
TEST(CAtan2LookUpTable, atan2LookUpTableHonorsRequestedLimits)
{
  const mrpt::math::CAtan2LookUpTable lut(-1.0, 1.0, -1.0, 1.0, 0.1);
  EXPECT_NEAR(lut.getXMin(), -1.0, 1e-9);
  EXPECT_NEAR(lut.getXMax(), 1.0, 1e-9);
  double a = 0;
  EXPECT_FALSE(lut.atan2(5.0, 5.0, a));
}

TEST(CAtan2LookUpTable, atan2LookUpTableAccuracy)
{
  const double res = 0.01;
  const mrpt::math::CAtan2LookUpTable lut(-2.0, 2.0, -2.0, 2.0, res);
  auto& rng = mrpt::random::getRandomGenerator();
  rng.randomize(1234);
  for (int i = 0; i < 2000; i++)
  {
    const double x = rng.drawUniform(-1.9, 1.9);
    const double y = rng.drawUniform(-1.9, 1.9);
    const double r = std::hypot(x, y);
    if (r < 0.2 || std::abs(x) < 0.02)
    {
      continue;  // near the origin or the branch cut
    }
    double a = 0;
    ASSERT_TRUE(lut.atan2(y, x, a));
    // Cell centers are at most res/sqrt(2) away from the query point
    EXPECT_NEAR(mrpt::math::wrapToPi(a - std::atan2(y, x)), 0.0, res / r);
  }
}
