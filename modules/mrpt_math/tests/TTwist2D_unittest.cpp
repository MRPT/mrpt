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
#include <mrpt/core/bits_math.h>
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TTwist2D.h>
#include <mrpt/serialization/CArchive.h>

#include <stdexcept>
#include <vector>

using mrpt::math::TTwist2D;

TEST(TTwist2D, componentAccess)
{
  TTwist2D t(1.0, 2.0, 3.0);
  EXPECT_EQ(t[0], 1.0);
  EXPECT_EQ(t[1], 2.0);
  EXPECT_EQ(t[2], 3.0);
  t[0] = 10.0;
  EXPECT_EQ(t.vx, 10.0);
  EXPECT_THROW(t[3], std::out_of_range);

  const TTwist2D ct(1.0, 2.0, 3.0);
  EXPECT_EQ(ct[2], 3.0);
  EXPECT_THROW(ct[3], std::out_of_range);
}

TEST(TTwist2D, vectorConversions)
{
  const std::vector<double> v{1.0, 2.0, 3.0};
  const TTwist2D t = TTwist2D::FromVector(v);
  EXPECT_EQ(t.omega, 3.0);

  std::vector<double> out;
  t.asVector(out);
  ASSERT_EQ(out.size(), 3U);
  EXPECT_EQ(out[1], 2.0);

  const auto out2 = t.asVector<std::vector<double>>();
  EXPECT_EQ(out2[2], 3.0);
}

TEST(TTwist2D, comparisonAndScaling)
{
  const TTwist2D a(1.0, 2.0, 3.0);
  TTwist2D b(1.0, 2.0, 3.0);

  EXPECT_TRUE(a == b);
  EXPECT_FALSE(a != b);

  b.omega = 4.0;
  EXPECT_FALSE(a == b);
  EXPECT_TRUE(a != b);

  TTwist2D c = a;
  c *= 2.0;
  EXPECT_EQ(c.vx, 2.0);
  EXPECT_EQ(c.omega, 6.0);

  // A twist times a time increment is a pose increment:
  const auto p = a * 0.5;
  EXPECT_NEAR(p.x, 0.5, 1e-12);
  EXPECT_NEAR(p.y, 1.0, 1e-12);
  EXPECT_NEAR(p.phi, 1.5, 1e-12);
}

TEST(TTwist2D, rotate)
{
  const TTwist2D t(1.0, 0.0, 0.5);

  const auto r90 = t.rotated(mrpt::DEG2RAD(90.0));
  EXPECT_NEAR(r90.vx, 0.0, 1e-12);
  EXPECT_NEAR(r90.vy, 1.0, 1e-12);
  // The angular velocity is not affected by a planar rotation:
  EXPECT_NEAR(r90.omega, 0.5, 1e-12);

  TTwist2D t2 = t;
  t2.rotate(0.0);
  EXPECT_NEAR(t2.vx, 1.0, 1e-12);
  EXPECT_NEAR(t2.vy, 0.0, 1e-12);
}

TEST(TTwist2D, stringConversion)
{
  const TTwist2D t(1.0, 2.0, mrpt::DEG2RAD(45.0));

  const std::string s = t.asString();
  EXPECT_FALSE(s.empty());

  const TTwist2D t2 = TTwist2D::FromString(s);
  EXPECT_NEAR(t2.vx, 1.0, 1e-4);
  EXPECT_NEAR(t2.vy, 2.0, 1e-4);
  EXPECT_NEAR(t2.omega, mrpt::DEG2RAD(45.0), 1e-4);

  TTwist2D t3;
  EXPECT_THROW(t3.fromString("not a matrix"), std::exception);
  EXPECT_THROW(t3.fromString("[1 2]"), std::exception);
}

TEST(TTwist2D, serialization)
{
  const TTwist2D t(1.0, 2.0, 3.0);

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << t;
  buf.Seek(0);
  TTwist2D t2;
  arch >> t2;
  EXPECT_TRUE(t == t2);
}
