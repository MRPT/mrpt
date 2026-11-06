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
#include <mrpt/math/CSplineInterpolator1D.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/serialization/CArchive.h>

using namespace mrpt::math;

TEST(CSplineInterpolator1D, DefaultCtorAndWrapFlag)
{
  CSplineInterpolator1D spl;
  EXPECT_FALSE(spl.getWrap2pi());
  spl.setWrap2pi(true);
  EXPECT_TRUE(spl.getWrap2pi());
}

TEST(CSplineInterpolator1D, VectorCtorAndSetXY)
{
  std::vector<double> xs{0, 1, 2, 3, 4};
  std::vector<double> ys{0, 1, 4, 9, 16};
  CSplineInterpolator1D spl(xs, ys);

  double y = 0;
  bool valid = false;
  spl.query(0, y, valid);
  EXPECT_TRUE(valid);
  EXPECT_NEAR(y, 0.0, 1e-9);

  // setXY without clearing previous content just overwrites overlapping keys
  spl.setXY(xs, ys, false);
}

TEST(CSplineInterpolator1D, AppendAndQueryExactMatch)
{
  CSplineInterpolator1D spl;
  spl.appendXY(0, 0);
  spl.appendXY(1, 1);
  spl.appendXY(2, 4);
  spl.appendXY(3, 9);
  spl.appendXY(4, 16);
  spl.appendXY(5, 25);

  double y = 0;
  bool valid = false;
  spl.query(2, y, valid);
  EXPECT_TRUE(valid);
  EXPECT_NEAR(y, 4.0, 1e-9);
}

TEST(CSplineInterpolator1D, QueryInterpolatedInterior)
{
  CSplineInterpolator1D spl;
  spl.appendXY(0, 0);
  spl.appendXY(1, 1);
  spl.appendXY(2, 4);
  spl.appendXY(3, 9);
  spl.appendXY(4, 16);
  spl.appendXY(5, 25);

  double y = 0;
  bool valid = false;
  spl.query(2.5, y, valid);
  EXPECT_TRUE(valid);
  // Should be reasonably close to the true quadratic value (6.25)
  EXPECT_NEAR(y, 6.25, 0.5);
}

TEST(CSplineInterpolator1D, QueryOutOfRangeIsInvalid)
{
  CSplineInterpolator1D spl;
  spl.appendXY(0, 0);
  spl.appendXY(1, 1);
  spl.appendXY(2, 4);
  spl.appendXY(3, 9);

  double y = 0;
  bool valid = false;

  // Before the first sample:
  spl.query(-10, y, valid);
  EXPECT_FALSE(valid);

  // After the last sample:
  spl.query(100, y, valid);
  EXPECT_FALSE(valid);
}

TEST(CSplineInterpolator1D, QueryNeedsFourSurroundingPoints)
{
  // With only 3 points, an interior query cannot gather 2 points on
  // each side, so it must report invalid.
  CSplineInterpolator1D spl;
  spl.appendXY(0, 0);
  spl.appendXY(1, 1);
  spl.appendXY(2, 4);

  double y = 0;
  bool valid = false;
  spl.query(0.5, y, valid);
  EXPECT_FALSE(valid);
}

TEST(CSplineInterpolator1D, QueryVector)
{
  CSplineInterpolator1D spl;
  spl.appendXY(0, 0);
  spl.appendXY(1, 1);
  spl.appendXY(2, 4);
  spl.appendXY(3, 9);
  spl.appendXY(4, 16);

  std::vector<double> xs{2, -10};
  std::vector<double> ys;
  const bool anyValid = spl.queryVector(xs, ys);
  EXPECT_TRUE(anyValid);
  ASSERT_EQ(ys.size(), 2u);
}

TEST(CSplineInterpolator1D, ClearRemovesData)
{
  CSplineInterpolator1D spl;
  spl.appendXY(0, 0);
  spl.appendXY(1, 1);
  spl.clear();

  double y = 0;
  bool valid = false;
  spl.query(0, y, valid);
  EXPECT_FALSE(valid);
}

TEST(CSplineInterpolator1D, SerializationRoundTrip)
{
  CSplineInterpolator1D spl(true /*wrap2pi*/);
  spl.appendXY(0, 0);
  spl.appendXY(1, 1);
  spl.appendXY(2, 4);

  mrpt::io::CMemoryStream membuf;
  auto arch = mrpt::serialization::archiveFrom(membuf);
  arch << spl;
  membuf.Seek(0);

  CSplineInterpolator1D spl2;
  arch >> spl2;

  EXPECT_TRUE(spl2.getWrap2pi());

  double y = 0;
  bool valid = false;
  spl2.query(1, y, valid);
  EXPECT_TRUE(valid);
  EXPECT_NEAR(y, 1.0, 1e-9);
}

// With wrap2pi, angles sampled across +-pi are unwrapped before
// interpolating: a linear angle sequence is reproduced exactly, and the
// result is in [-pi,pi[.
TEST(CSplineInterpolator1D, Wrap2piAcrossPi)
{
  const auto theta = [](double t) { return M_PI - 0.3 + 0.2 * t; };
  CSplineInterpolator1D spl(true);
  for (int i = 0; i < 6; i++)
  {
    spl.appendXY(i, mrpt::math::wrapToPi(theta(i)));
  }
  for (const double t : {1.2, 1.5, 2.0, 2.7, 3.9})
  {
    double y = 0;
    bool valid = false;
    spl.query(t, y, valid);
    ASSERT_TRUE(valid) << "t=" << t;
    EXPECT_NEAR(mrpt::math::wrapToPi(y - theta(t)), 0.0, 1e-12) << "t=" << t;
    EXPECT_GE(y, -M_PI);
    EXPECT_LT(y, M_PI);
  }
  // Same function and samples without wrapping (continuous input):
  CSplineInterpolator1D spl2(false);
  for (int i = 0; i < 6; i++)
  {
    spl2.appendXY(i, theta(i));
  }
  double y2 = 0;
  bool valid2 = false;
  spl2.query(2.7, y2, valid2);
  ASSERT_TRUE(valid2);
  EXPECT_NEAR(y2, theta(2.7), 1e-12);
}
