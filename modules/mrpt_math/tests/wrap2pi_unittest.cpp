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

#include <cmath>
#include <vector>

using namespace mrpt;
using namespace std;

TEST(Wrap2PI_tests, angDistance)
{
  using mrpt::math::angDistance;

  EXPECT_NEAR(angDistance(0.0, 1.0), 1.0, 1e-10);
  EXPECT_NEAR(angDistance(1.0, 1.0), 0.0, 1e-10);
  EXPECT_NEAR(angDistance(1.0, 0.0), -1.0, 1e-10);

  EXPECT_NEAR(angDistance(-(M_PI - 0.1), (M_PI - 0.1)), -0.2, 1e-6);
  EXPECT_NEAR(angDistance((M_PI - 0.1), -(M_PI - 0.1)), +0.2, 1e-6);
}

TEST(Wrap2PI_tests, wrapToPiGenericValues)
{
  for (double a = -20.0; a <= 20.0; a += 0.37)
  {
    const double w = mrpt::math::wrapToPi(a);
    EXPECT_GE(w, -M_PI);
    EXPECT_LT(w, M_PI);
    EXPECT_NEAR(std::sin(w), std::sin(a), 1e-12);
    EXPECT_NEAR(std::cos(w), std::cos(a), 1e-12);
  }
}

TEST(Wrap2PI_tests, wrapToPiRangeEnds)
{
  using mrpt::math::wrapToPi;
  // Range is [-pi, pi[: both +pi and -pi map to -pi.
  EXPECT_DOUBLE_EQ(wrapToPi(M_PI), -M_PI);
  EXPECT_DOUBLE_EQ(wrapToPi(-M_PI), -M_PI);
  EXPECT_DOUBLE_EQ(wrapToPi(3 * M_PI), -M_PI);
  EXPECT_LT(wrapToPi(-M_PI - 1e-15), M_PI);
}

TEST(Wrap2PI_tests, wrapTo2PiRange)
{
  using mrpt::math::wrapTo2Pi;
  EXPECT_LT(wrapTo2Pi(-2 * M_PI), 2 * M_PI);
  EXPECT_GE(wrapTo2Pi(-2 * M_PI), 0.0);
  // Tiny negative values must not round up to 2pi:
  EXPECT_LT(wrapTo2Pi(-1e-17), 2 * M_PI);
  EXPECT_GE(wrapTo2Pi(-1e-17), 0.0);
  EXPECT_NEAR(wrapTo2Pi(-0.5), 2 * M_PI - 0.5, 1e-15);
  EXPECT_NEAR(wrapTo2Pi(7.0), 7.0 - 2 * M_PI, 1e-15);
  EXPECT_DOUBLE_EQ(wrapTo2Pi(2 * M_PI), 0.0);
}

TEST(Wrap2PI_tests, angDistanceDocumentedExamples)
{
  using mrpt::math::angDistance;
  EXPECT_NEAR(angDistance(0.0, M_PI), -M_PI, 1e-12);
  EXPECT_NEAR(angDistance(M_PI, 0.0), M_PI, 1e-12);
  EXPECT_NEAR(angDistance(-3.1, 3.1), -(2 * M_PI - 6.2), 1e-12);
  EXPECT_NEAR(angDistance(3.1, -3.1), 2 * M_PI - 6.2, 1e-12);
}

TEST(Wrap2PI_tests, unwrap2PiSequenceMultipleTurns)
{
  std::vector<double> x;
  for (int i = 0; i < 12; i++)
  {
    x.push_back(mrpt::math::wrapToPi(2.0 * i));
  }
  mrpt::math::unwrap2PiSequence(x);
  for (size_t i = 1; i < x.size(); i++)
  {
    EXPECT_LE(std::abs(x[i] - x[i - 1]), M_PI) << "i=" << i;
    EXPECT_NEAR(x[i] - x[0], 2.0 * static_cast<double>(i), 1e-9) << "i=" << i;
  }
  // Same, turning in the opposite direction:
  for (size_t i = 0; i < x.size(); i++)
  {
    x[i] = mrpt::math::wrapToPi(-2.5 * static_cast<double>(i));
  }
  mrpt::math::unwrap2PiSequence(x);
  for (size_t i = 1; i < x.size(); i++)
  {
    EXPECT_NEAR(x[i] - x[0], -2.5 * static_cast<double>(i), 1e-9) << "i=" << i;
  }
}

TEST(Wrap2PI_tests, unwrap2PiSequenceSingleCrossing)
{
  std::vector<double> x = {3.0, -3.0, -2.7, 3.1, 2.9};
  mrpt::math::unwrap2PiSequence(x);
  for (size_t i = 1; i < x.size(); i++)
  {
    EXPECT_LE(std::abs(x[i] - x[i - 1]), M_PI) << "i=" << i;
  }
  EXPECT_NEAR(x[1], -3.0 + 2 * M_PI, 1e-12);
  std::vector<double> empty;
  mrpt::math::unwrap2PiSequence(empty);
  EXPECT_TRUE(empty.empty());
}
