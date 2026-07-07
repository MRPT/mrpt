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
#include <mrpt/math/TSegment2D.h>
#include <mrpt/math/TSegment3D.h>
#include <mrpt/serialization/CArchive.h>

using namespace mrpt::math;

TEST(TSegment3D, Generate2DObject)
{
  const TSegment3D s3(TPoint3D(1, 2, 0), TPoint3D(3, 4, 0));
  TSegment2D s2;
  s3.generate2DObject(s2);
  EXPECT_NEAR(s2.point1.x, 1.0, 1e-9);
  EXPECT_NEAR(s2.point2.y, 4.0, 1e-9);
}

TEST(TSegment3D, Length)
{
  const TSegment3D s(TPoint3D(0, 0, 0), TPoint3D(3, 4, 0));
  EXPECT_NEAR(s.length(), 5.0, 1e-9);
}

TEST(TSegment3D, DistanceToPoint)
{
  const TSegment3D s(TPoint3D(0, 0, 0), TPoint3D(2, 0, 0));
  EXPECT_NEAR(s.distance(TPoint3D(1, 1, 0)), 1.0, 1e-9);
  // TSegment3D::distance() is the minimum of the distance to each endpoint
  // and the distance to the *infinite* line through the segment, so points
  // beyond either endpoint but still on that line report 0, not the
  // distance to the nearest endpoint.
  EXPECT_NEAR(s.distance(TPoint3D(-1, 0, 0)), 0.0, 1e-9);
  EXPECT_NEAR(s.distance(TPoint3D(3, 0, 0)), 0.0, 1e-9);
}

TEST(TSegment3D, DistanceToSegmentParallel)
{
  const TSegment3D s1(TPoint3D(0, 0, 0), TPoint3D(2, 0, 0));
  const TSegment3D s2(TPoint3D(0, 3, 0), TPoint3D(2, 3, 0));
  EXPECT_NEAR(s1.distance(s2), 3.0, 1e-6);
}

TEST(TSegment3D, DistanceToSegmentSkew)
{
  // Two perpendicular segments in different planes, not crossing.
  const TSegment3D s1(TPoint3D(0, 0, 0), TPoint3D(2, 0, 0));
  const TSegment3D s2(TPoint3D(1, 1, 1), TPoint3D(1, -1, 1));
  EXPECT_NEAR(s1.distance(s2), 1.0, 1e-6);
}

TEST(TSegment3D, DistanceToSegmentEndpointVisible)
{
  // s2 is entirely "beyond" the end of s1, so the t=1 edge should be visible.
  const TSegment3D s1(TPoint3D(0, 0, 0), TPoint3D(1, 0, 0));
  const TSegment3D s2(TPoint3D(3, 1, 0), TPoint3D(3, 2, 0));
  EXPECT_NEAR(s1.distance(s2), std::sqrt(2.0 * 2.0 + 1.0 * 1.0), 1e-6);
}

TEST(TSegment3D, Contains)
{
  const TSegment3D s(TPoint3D(0, 0, 0), TPoint3D(2, 0, 0));
  EXPECT_TRUE(s.contains(TPoint3D(1, 0, 0)));
  EXPECT_TRUE(s.contains(TPoint3D(0, 0, 0)));
  EXPECT_FALSE(s.contains(TPoint3D(3, 0, 0)));
  EXPECT_FALSE(s.contains(TPoint3D(1, 1, 0)));
}

TEST(TSegment3D, LessThanOperator)
{
  const TSegment3D s1(TPoint3D(0, 0, 0), TPoint3D(1, 0, 0));
  const TSegment3D s2(TPoint3D(0, 0, 0), TPoint3D(2, 0, 0));
  const TSegment3D s3(TPoint3D(1, 0, 0), TPoint3D(0, 0, 0));

  EXPECT_TRUE(s1 < s2);
  EXPECT_FALSE(s2 < s1);
  EXPECT_TRUE(s1 < s3);
}

TEST(TSegment3D, SerializationRoundTrip)
{
  const TSegment3D s(TPoint3D(1, 2, 3), TPoint3D(4, 5, 6));

  mrpt::io::CMemoryStream membuf;
  auto arch = mrpt::serialization::archiveFrom(membuf);
  arch << s;
  membuf.Seek(0);

  TSegment3D s2;
  arch >> s2;

  EXPECT_EQ(s.point1, s2.point1);
  EXPECT_EQ(s.point2, s2.point2);
}

TEST(TSegment3D, AsStringAndStreamOperator)
{
  const TSegment3D s(TPoint3D(1, 2, 3), TPoint3D(4, 5, 6));
  EXPECT_FALSE(s.asString().empty());

  std::ostringstream ss;
  ss << s;
  EXPECT_FALSE(ss.str().empty());
}
