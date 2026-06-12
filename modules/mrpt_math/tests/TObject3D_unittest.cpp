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
#include <mrpt/math/TObject2D.h>
#include <mrpt/math/TObject3D.h>
#include <mrpt/serialization/CArchive.h>

using namespace mrpt::math;

namespace
{
template <typename T>
T roundTripSerialize(const T& obj)
{
  mrpt::io::CMemoryStream membuf;
  auto arch = mrpt::serialization::archiveFrom(membuf);
  arch << obj;
  membuf.Seek(0);
  T obj2;
  arch >> obj2;
  return obj2;
}
}  // namespace

TEST(TObject3D, EmptySerialization)
{
  TObject3D obj;
  EXPECT_TRUE(obj.empty());

  TObject3D obj2 = roundTripSerialize(obj);
  EXPECT_TRUE(obj2.empty());
  EXPECT_EQ(obj.asString(), obj2.asString());
}

TEST(TObject3D, PointSerializationAndGenerate2D)
{
  TObject3D obj = TObject3D::From(TPoint3D(1, 2, 3));
  ASSERT_TRUE(obj.isPoint());

  TObject3D obj2 = roundTripSerialize(obj);
  ASSERT_TRUE(obj2.isPoint());
  EXPECT_EQ(obj.getAs<TPoint3D>(), obj2.getAs<TPoint3D>());

  TObject2D obj2d = obj.generate2DObject();
  ASSERT_TRUE(obj2d.isPoint());
  EXPECT_EQ(obj2d.getAs<TPoint2D>().x, 1);
  EXPECT_EQ(obj2d.getAs<TPoint2D>().y, 2);

  EXPECT_FALSE(obj.asString().empty());
}

TEST(TObject3D, SegmentSerializationAndGenerate2D)
{
  TObject3D obj = TObject3D::From(TSegment3D(TPoint3D(0, 0, 0), TPoint3D(1, 1, 1)));
  ASSERT_TRUE(obj.isSegment());

  TObject3D obj2 = roundTripSerialize(obj);
  ASSERT_TRUE(obj2.isSegment());
  EXPECT_EQ(obj.getAs<TSegment3D>(), obj2.getAs<TSegment3D>());

  TObject2D obj2d = obj.generate2DObject();
  EXPECT_TRUE(obj2d.isSegment());
}

TEST(TObject3D, LineSerializationAndGenerate2D)
{
  TObject3D obj = TObject3D::From(TLine3D::FromTwoPoints({0, 0, 0}, {1, 0, 0}));
  ASSERT_TRUE(obj.isLine());

  TObject3D obj2 = roundTripSerialize(obj);
  ASSERT_TRUE(obj2.isLine());

  TObject2D obj2d = obj.generate2DObject();
  EXPECT_TRUE(obj2d.isLine());
}

TEST(TObject3D, PolygonSerializationAndGenerate2D)
{
  TPolygon3D poly{TPoint3D(0, 0, 0), TPoint3D(1, 0, 0), TPoint3D(0, 1, 0)};
  TObject3D obj = TObject3D::From(poly);
  ASSERT_TRUE(obj.isPolygon());

  TObject3D obj2 = roundTripSerialize(obj);
  ASSERT_TRUE(obj2.isPolygon());
  EXPECT_EQ(obj.getAs<TPolygon3D>().size(), obj2.getAs<TPolygon3D>().size());

  TObject2D obj2d = obj.generate2DObject();
  EXPECT_TRUE(obj2d.isPolygon());
}

TEST(TObject3D, PlaneSerializationAndGenerate2DThrows)
{
  TObject3D obj = TObject3D::From(TPlane::FromPointAndNormal({0, 0, 0}, {0, 0, 1}));
  ASSERT_TRUE(obj.isPlane());

  TObject3D obj2 = roundTripSerialize(obj);
  ASSERT_TRUE(obj2.isPlane());

  // A 3D plane cannot be cast down to 2D.
  EXPECT_THROW((void)obj.generate2DObject(), std::exception);

  EXPECT_FALSE(obj.asString().empty());
}

TEST(TObject3D, GetterHelpers)
{
  TObject3D obj = TObject3D::From(TPoint3D(1, 2, 3));

  TPoint3D pt;
  EXPECT_TRUE(obj.getPoint(pt));
  EXPECT_EQ(pt, TPoint3D(1, 2, 3));

  TSegment3D seg;
  EXPECT_FALSE(obj.getSegment(seg));

  TLine3D line;
  EXPECT_FALSE(obj.getLine(line));

  TPolygon3D poly;
  EXPECT_FALSE(obj.getPolygon(poly));

  TPlane plane;
  EXPECT_FALSE(obj.getPlane(plane));
}
