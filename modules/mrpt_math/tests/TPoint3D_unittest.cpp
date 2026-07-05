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
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/serialization/CArchive.h>

using namespace mrpt::math;

TEST(TPoint3D, CtorFromTPoint2D)
{
  const TPoint2D p2(1, 2);
  const TPoint3D p3(p2);
  EXPECT_NEAR(p3.x, 1.0, 1e-9);
  EXPECT_NEAR(p3.y, 2.0, 1e-9);
  EXPECT_NEAR(p3.z, 0.0, 1e-9);
}

TEST(TPoint3D, CtorFromTPose2D)
{
  const TPose2D pose(1, 2, 0.5);
  const TPoint3D p3(pose);
  EXPECT_NEAR(p3.x, 1.0, 1e-9);
  EXPECT_NEAR(p3.y, 2.0, 1e-9);
  EXPECT_NEAR(p3.z, 0.0, 1e-9);
}

TEST(TPoint3D, CtorFromTPose3D)
{
  const TPose3D pose(1, 2, 3, 0.1, 0.2, 0.3);
  const TPoint3D p3(pose);
  EXPECT_NEAR(p3.x, 1.0, 1e-9);
  EXPECT_NEAR(p3.y, 2.0, 1e-9);
  EXPECT_NEAR(p3.z, 3.0, 1e-9);
}

TEST(TPoint3D, LessThanOperator)
{
  EXPECT_TRUE(TPoint3D(1, 0, 0) < TPoint3D(2, 0, 0));
  EXPECT_FALSE(TPoint3D(2, 0, 0) < TPoint3D(1, 0, 0));
  EXPECT_TRUE(TPoint3D(1, 1, 0) < TPoint3D(1, 2, 0));
  EXPECT_TRUE(TPoint3D(1, 1, 1) < TPoint3D(1, 1, 2));
  EXPECT_FALSE(TPoint3D(1, 1, 1) < TPoint3D(1, 1, 1));
}

TEST(TPoint3D, FromStringValid)
{
  TPoint3D p;
  p.fromString("[1.0 2.0 3.0]");
  EXPECT_NEAR(p.x, 1.0, 1e-9);
  EXPECT_NEAR(p.y, 2.0, 1e-9);
  EXPECT_NEAR(p.z, 3.0, 1e-9);
}

TEST(TPoint3D, FromStringMalformedThrows)
{
  TPoint3D p;
  EXPECT_THROW(p.fromString("not a matrix"), std::exception);
  EXPECT_THROW(p.fromString("[1.0 2.0]"), std::exception);
}

TEST(TPoint3D, XYZfRGBu8Serialization)
{
  TPointXYZfRGBu8 p;
  p.pt = TPoint3Df(1.0f, 2.0f, 3.0f);
  p.r = 10;
  p.g = 20;
  p.b = 30;

  mrpt::io::CMemoryStream membuf;
  auto arch = mrpt::serialization::archiveFrom(membuf);
  arch << p;
  membuf.Seek(0);

  TPointXYZfRGBu8 p2;
  arch >> p2;

  EXPECT_NEAR(p2.pt.x, 1.0f, 1e-6);
  EXPECT_EQ(p2.r, 10);
  EXPECT_EQ(p2.g, 20);
  EXPECT_EQ(p2.b, 30);
}

TEST(TPoint3D, XYZfRGBAu8Serialization)
{
  TPointXYZfRGBAu8 p;
  p.pt = TPoint3Df(1.0f, 2.0f, 3.0f);
  p.r = 10;
  p.g = 20;
  p.b = 30;
  p.a = 40;

  mrpt::io::CMemoryStream membuf;
  auto arch = mrpt::serialization::archiveFrom(membuf);
  arch << p;
  membuf.Seek(0);

  TPointXYZfRGBAu8 p2;
  arch >> p2;

  EXPECT_NEAR(p2.pt.z, 3.0f, 1e-6);
  EXPECT_EQ(p2.r, 10);
  EXPECT_EQ(p2.a, 40);
}
