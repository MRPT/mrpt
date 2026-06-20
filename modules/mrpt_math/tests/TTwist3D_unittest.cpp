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
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/TTwist3D.h>
#include <mrpt/serialization/CArchive.h>

using namespace mrpt::math;

TEST(TTwist3D, ComponentAccess)
{
  TTwist3D tw(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
  EXPECT_DOUBLE_EQ(tw[0], 1.0);
  EXPECT_DOUBLE_EQ(tw[1], 2.0);
  EXPECT_DOUBLE_EQ(tw[2], 3.0);
  EXPECT_DOUBLE_EQ(tw[3], 4.0);
  EXPECT_DOUBLE_EQ(tw[4], 5.0);
  EXPECT_DOUBLE_EQ(tw[5], 6.0);

  EXPECT_DOUBLE_EQ(tw(0, 0), 1.0);
  EXPECT_DOUBLE_EQ(tw(5, 0), 6.0);

  EXPECT_THROW(tw[6], std::out_of_range);

  const TTwist3D ctw = tw;
  EXPECT_THROW(ctw[6], std::out_of_range);

  tw[0] = 10.0;
  EXPECT_DOUBLE_EQ(tw.vx, 10.0);
}

TEST(TTwist3D, EqualityOperators)
{
  TTwist3D a(1, 2, 3, 4, 5, 6);
  TTwist3D b(1, 2, 3, 4, 5, 6);
  TTwist3D c(1, 2, 3, 4, 5, 7);

  EXPECT_TRUE(a == b);
  EXPECT_FALSE(a != b);
  EXPECT_FALSE(a == c);
  EXPECT_TRUE(a != c);
}

TEST(TTwist3D, ScaleOperator)
{
  TTwist3D tw(1, 2, 3, 4, 5, 6);
  tw *= 2.0;
  EXPECT_TRUE(tw == TTwist3D(2, 4, 6, 8, 10, 12));
}

TEST(TTwist3D, VectorConversions)
{
  std::vector<double> v = {1, 2, 3, 4, 5, 6};
  auto tw = TTwist3D::FromVector(v);
  EXPECT_TRUE(tw == TTwist3D(1, 2, 3, 4, 5, 6));

  std::vector<double> out;
  tw.asVector(out);
  EXPECT_EQ(out, v);

  auto out2 = tw.asVector<std::vector<double>>();
  EXPECT_EQ(out2, v);

  TTwist3D tw2;
  tw2.fromVector(v);
  EXPECT_TRUE(tw2 == tw);
}

TEST(TTwist3D, AsStringAndFromString)
{
  TTwist3D tw(1.0, 2.0, 3.0, 0.0, 0.0, 0.0);
  std::string s;
  tw.asString(s);
  EXPECT_FALSE(s.empty());

  TTwist3D tw2 = TTwist3D::FromString(s);
  EXPECT_NEAR(tw2.vx, tw.vx, 1e-6);
  EXPECT_NEAR(tw2.vy, tw.vy, 1e-6);
  EXPECT_NEAR(tw2.vz, tw.vz, 1e-6);
  EXPECT_NEAR(tw2.wx, tw.wx, 1e-6);
  EXPECT_NEAR(tw2.wy, tw.wy, 1e-6);
  EXPECT_NEAR(tw2.wz, tw.wz, 1e-6);

  EXPECT_THROW(TTwist3D::FromString("malformed"), std::exception);
  EXPECT_THROW(TTwist3D::FromString("[1 2 3]"), std::exception);
}

TEST(TTwist3D, RotateIdentity)
{
  TTwist3D tw(1.0, 2.0, 3.0, 0.1, 0.2, 0.3);
  TTwist3D rotated = tw.rotated(TPose3D::Identity());
  EXPECT_TRUE(rotated == tw);

  TTwist3D tw2 = tw;
  tw2.rotate(TPose3D::Identity());
  EXPECT_TRUE(tw2 == tw);
}

TEST(TTwist3D, RotateYaw90)
{
  // 90 deg yaw rotation: x axis maps to y axis
  TTwist3D tw(1.0, 0.0, 0.0, 0.0, 0.0, 1.0);
  TPose3D rot(0, 0, 0, M_PI / 2, 0, 0);
  TTwist3D rotated = tw.rotated(rot);

  EXPECT_NEAR(rotated.vx, 0.0, 1e-9);
  EXPECT_NEAR(rotated.vy, 1.0, 1e-9);
  EXPECT_NEAR(rotated.vz, 0.0, 1e-9);
  EXPECT_NEAR(rotated.wx, 0.0, 1e-9);
  EXPECT_NEAR(rotated.wy, 0.0, 1e-9);
  EXPECT_NEAR(rotated.wz, 1.0, 1e-9);
}

TEST(TTwist3D, Serialization)
{
  TTwist3D tw(1.5, -2.5, 3.5, 0.1, -0.2, 0.3);

  mrpt::io::CMemoryStream membuf;
  auto arch = mrpt::serialization::archiveFrom(membuf);
  arch << tw;

  membuf.Seek(0);
  TTwist3D tw2;
  arch >> tw2;

  EXPECT_TRUE(tw == tw2);
}
