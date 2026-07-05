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
#include <mrpt/math/TPose3DQuat.h>

using namespace mrpt::math;

TEST(TPose3DQuat, DefaultCtorIsIdentity)
{
  TPose3DQuat p;
  EXPECT_EQ(p.x, 0.0);
  EXPECT_EQ(p.y, 0.0);
  EXPECT_EQ(p.z, 0.0);
  EXPECT_EQ(p.qr, 1.0);
  EXPECT_EQ(p.qx, 0.0);
  EXPECT_EQ(p.qy, 0.0);
  EXPECT_EQ(p.qz, 0.0);
}

TEST(TPose3DQuat, CoordinateCtorAndIndexing)
{
  TPose3DQuat p(1, 2, 3, 0.5, 0.5, 0.5, 0.5);
  EXPECT_EQ(p[0], 1);
  EXPECT_EQ(p[1], 2);
  EXPECT_EQ(p[2], 3);
  EXPECT_EQ(p[3], 0.5);
  EXPECT_EQ(p[4], 0.5);
  EXPECT_EQ(p[5], 0.5);
  EXPECT_EQ(p[6], 0.5);

  const TPose3DQuat& cp = p;
  EXPECT_EQ(cp[0], 1);
  EXPECT_EQ(cp[6], 0.5);

  p[0] = 10;
  EXPECT_EQ(p.x, 10);

  EXPECT_THROW(p[7], std::out_of_range);
  EXPECT_THROW(cp[7], std::out_of_range);
}

TEST(TPose3DQuat, Norm)
{
  TPose3DQuat p(3, 4, 0, 1, 0, 0, 0);
  EXPECT_NEAR(p.norm(), 5.0, 1e-9);
}

TEST(TPose3DQuat, AsVector)
{
  TPose3DQuat p(1, 2, 3, 0.1, 0.2, 0.3, 0.4);
  std::vector<double> v;
  p.asVector(v);
  ASSERT_EQ(v.size(), 7u);
  for (size_t i = 0; i < 7; i++)
  {
    EXPECT_EQ(v[i], p[i]);
  }
}

TEST(TPose3DQuat, AsStringAndFromString)
{
  TPose3DQuat p(1, 2, 3, 1, 0, 0, 0);
  std::string s;
  p.asString(s);
  EXPECT_FALSE(s.empty());

  auto p2 = TPose3DQuat::FromString("[1.0 2.0 3.0 1.0 0.0 0.0 0.0]");
  EXPECT_EQ(p2.x, 1.0);
  EXPECT_EQ(p2.y, 2.0);
  EXPECT_EQ(p2.z, 3.0);
  EXPECT_EQ(p2.qr, 1.0);
}

TEST(TPose3DQuat, FromStringMalformedThrows)
{
  EXPECT_THROW(TPose3DQuat::FromString("this is not a matrix"), std::exception);
  // wrong size: only 3 columns instead of 7
  EXPECT_THROW(TPose3DQuat::FromString("[1.0 2.0 3.0]"), std::exception);
}

TEST(TPose3DQuat, EqualityOperators)
{
  TPose3DQuat p1(1, 2, 3, 1, 0, 0, 0);
  TPose3DQuat p2(1, 2, 3, 1, 0, 0, 0);
  TPose3DQuat p3(1, 2, 3, 0, 1, 0, 0);

  EXPECT_TRUE(p1 == p2);
  EXPECT_FALSE(p1 != p2);
  EXPECT_FALSE(p1 == p3);
  EXPECT_TRUE(p1 != p3);
}
