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

TEST(TObject2D, EmptySerialization)
{
  TObject2D obj;
  EXPECT_TRUE(obj.empty());

  TObject2D obj2 = roundTripSerialize(obj);
  EXPECT_TRUE(obj2.empty());
  EXPECT_EQ(obj.asString(), obj2.asString());

  TObject3D obj3d = obj.generate3DObject();
  EXPECT_TRUE(obj3d.empty());
}

TEST(TObject2D, PointSerializationAndGenerate3D)
{
  TObject2D obj = TObject2D::From(TPoint2D(1, 2));
  ASSERT_TRUE(obj.isPoint());

  TObject2D obj2 = roundTripSerialize(obj);
  ASSERT_TRUE(obj2.isPoint());
  EXPECT_EQ(obj.getAs<TPoint2D>(), obj2.getAs<TPoint2D>());

  TObject3D obj3d = obj.generate3DObject();
  ASSERT_TRUE(obj3d.isPoint());
  EXPECT_EQ(obj3d.getAs<TPoint3D>().x, 1);
  EXPECT_EQ(obj3d.getAs<TPoint3D>().y, 2);

  EXPECT_FALSE(obj.asString().empty());
}

TEST(TObject2D, SegmentSerializationAndGenerate3D)
{
  TObject2D obj = TObject2D::From(TSegment2D(TPoint2D(0, 0), TPoint2D(1, 1)));
  ASSERT_TRUE(obj.isSegment());

  TObject2D obj2 = roundTripSerialize(obj);
  ASSERT_TRUE(obj2.isSegment());
  EXPECT_EQ(obj.getAs<TSegment2D>(), obj2.getAs<TSegment2D>());

  TObject3D obj3d = obj.generate3DObject();
  EXPECT_TRUE(obj3d.isSegment());
}

TEST(TObject2D, LineSerializationAndGenerate3D)
{
  TObject2D obj = TObject2D::From(TLine2D::FromTwoPoints({0, 0}, {1, 0}));
  ASSERT_TRUE(obj.isLine());

  TObject2D obj2 = roundTripSerialize(obj);
  ASSERT_TRUE(obj2.isLine());

  TObject3D obj3d = obj.generate3DObject();
  EXPECT_TRUE(obj3d.isLine());
}

TEST(TObject2D, PolygonSerializationAndGenerate3D)
{
  TPolygon2D poly{TPoint2D(0, 0), TPoint2D(1, 0), TPoint2D(0, 1)};
  TObject2D obj = TObject2D::From(poly);
  ASSERT_TRUE(obj.isPolygon());

  TObject2D obj2 = roundTripSerialize(obj);
  ASSERT_TRUE(obj2.isPolygon());
  EXPECT_EQ(obj.getAs<TPolygon2D>().size(), obj2.getAs<TPolygon2D>().size());

  TObject3D obj3d = obj.generate3DObject();
  EXPECT_TRUE(obj3d.isPolygon());
}

TEST(TObject2D, GetterHelpers)
{
  TObject2D obj = TObject2D::From(TPoint2D(1, 2));

  TPoint2D pt;
  EXPECT_TRUE(obj.getPoint(pt));
  EXPECT_EQ(pt, TPoint2D(1, 2));

  TSegment2D seg;
  EXPECT_FALSE(obj.getSegment(seg));

  TLine2D line;
  EXPECT_FALSE(obj.getLine(line));

  TPolygon2D poly;
  EXPECT_FALSE(obj.getPolygon(poly));
}

TEST(TObject2D, DeserializeInvalidTypeIndexThrows)
{
  mrpt::io::CMemoryStream membuf;
  auto arch = mrpt::serialization::archiveFrom(membuf);
  arch.WriteAs<uint8_t>(255);
  membuf.Seek(0);

  TObject2D obj;
  EXPECT_THROW(arch >> obj, std::exception);
}
