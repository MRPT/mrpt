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

// Exhaustive coverage of the type-dispatch tables in
// math::intersect(const TObject2D&, const TObject2D&, TObject2D&) and
// math::intersect(const TObject3D&, const TObject3D&, TObject3D&), which
// otherwise are only partially exercised by higher-level tests.

#include <gtest/gtest.h>
#include <mrpt/math/TObject2D.h>
#include <mrpt/math/TObject3D.h>
#include <mrpt/math/geometry.h>

using namespace mrpt::math;

namespace
{
// All of these 2D primitives mutually intersect at, or contain, (1,1):
const TPoint2D kPt2{1, 1};
const TLine2D kHorizLine2 = TLine2D::FromTwoPoints({-5, 1}, {5, 1});
const TLine2D kVertLine2 = TLine2D::FromTwoPoints({1, -5}, {1, 5});
const TSegment2D kHorizSeg2(TPoint2D(-1, 1), TPoint2D(3, 1));
const TSegment2D kVertSeg2(TPoint2D(1, -1), TPoint2D(1, 3));
const TPolygon2D kSquare2{
    {0, 0},
    {2, 0},
    {2, 2},
    {0, 2}
};

// All of these 3D primitives mutually intersect at, or contain, (1,1,0),
// and all but kCrossSeg3/lines lie within the Z=0 plane.
const TPoint3D kPt3{1, 1, 0};
const TLine3D kHorizLine3 = TLine3D::FromTwoPoints({-5, 1, 0}, {5, 1, 0});
const TLine3D kVertLine3 = TLine3D::FromTwoPoints({1, -5, 0}, {1, 5, 0});
const TSegment3D kHorizSeg3(TPoint3D(-1, 1, 0), TPoint3D(3, 1, 0));
const TSegment3D kVertSeg3(TPoint3D(1, -1, 0), TPoint3D(1, 3, 0));
// Crosses the Z=0 plane at the origin (not fully contained in it):
const TSegment3D kCrossSeg3(TPoint3D(0, 0, -1), TPoint3D(0, 0, 1));
const TPolygon3D kSquare3{
    {0, 0, 0},
    {2, 0, 0},
    {2, 2, 0},
    {0, 2, 0}
};
const TPlane kPlaneZ0 = TPlane::FromPointAndNormal({0, 0, 0}, {0, 0, 1});
}  // namespace

TEST(GeometryObjectDispatch2D, PointVsAll)
{
  TObject2D obj;
  EXPECT_TRUE(mrpt::math::intersect(TObject2D::From(kPt2), TObject2D::From(kPt2), obj))
      << "point-point";
  EXPECT_TRUE(mrpt::math::intersect(TObject2D::From(kPt2), TObject2D::From(kHorizSeg2), obj))
      << "point-segment";
  EXPECT_TRUE(mrpt::math::intersect(TObject2D::From(kPt2), TObject2D::From(kHorizLine2), obj))
      << "point-line";
  EXPECT_TRUE(mrpt::math::intersect(TObject2D::From(kPt2), TObject2D::From(kSquare2), obj))
      << "point-polygon";
}

TEST(GeometryObjectDispatch2D, SegmentVsAll)
{
  TObject2D obj;
  EXPECT_TRUE(mrpt::math::intersect(TObject2D::From(kHorizSeg2), TObject2D::From(kPt2), obj))
      << "segment-point";
  EXPECT_TRUE(mrpt::math::intersect(TObject2D::From(kHorizSeg2), TObject2D::From(kVertSeg2), obj))
      << "segment-segment";
  EXPECT_TRUE(mrpt::math::intersect(TObject2D::From(kHorizSeg2), TObject2D::From(kVertLine2), obj))
      << "segment-line";
  EXPECT_TRUE(mrpt::math::intersect(TObject2D::From(kHorizSeg2), TObject2D::From(kSquare2), obj))
      << "segment-polygon";
}

TEST(GeometryObjectDispatch2D, LineVsAll)
{
  TObject2D obj;
  EXPECT_TRUE(mrpt::math::intersect(TObject2D::From(kHorizLine2), TObject2D::From(kPt2), obj))
      << "line-point";
  EXPECT_TRUE(mrpt::math::intersect(TObject2D::From(kHorizLine2), TObject2D::From(kVertSeg2), obj))
      << "line-segment";
  EXPECT_TRUE(mrpt::math::intersect(TObject2D::From(kHorizLine2), TObject2D::From(kVertLine2), obj))
      << "line-line";
  EXPECT_TRUE(mrpt::math::intersect(TObject2D::From(kHorizLine2), TObject2D::From(kSquare2), obj))
      << "line-polygon";
}

TEST(GeometryObjectDispatch2D, PolygonVsAll)
{
  TObject2D obj;
  EXPECT_TRUE(mrpt::math::intersect(TObject2D::From(kSquare2), TObject2D::From(kPt2), obj))
      << "polygon-point";
  EXPECT_TRUE(mrpt::math::intersect(TObject2D::From(kSquare2), TObject2D::From(kVertSeg2), obj))
      << "polygon-segment";
  EXPECT_TRUE(mrpt::math::intersect(TObject2D::From(kSquare2), TObject2D::From(kVertLine2), obj))
      << "polygon-line";
  const TPolygon2D overlapping{
      {1, 1},
      {3, 1},
      {3, 3},
      {1, 3}
  };
  EXPECT_TRUE(mrpt::math::intersect(TObject2D::From(kSquare2), TObject2D::From(overlapping), obj))
      << "polygon-polygon";
}

TEST(GeometryObjectDispatch2D, NoIntersectionReturnsFalse)
{
  TObject2D obj;
  const TPoint2D farPt{100, 100};
  EXPECT_FALSE(mrpt::math::intersect(TObject2D::From(kPt2), TObject2D::From(farPt), obj));
}

TEST(GeometryObjectDispatch3D, PointVsAll)
{
  TObject3D obj;
  EXPECT_TRUE(mrpt::math::intersect(TObject3D::From(kPt3), TObject3D::From(kPt3), obj))
      << "point-point";
  EXPECT_TRUE(mrpt::math::intersect(TObject3D::From(kPt3), TObject3D::From(kHorizSeg3), obj))
      << "point-segment";
  EXPECT_TRUE(mrpt::math::intersect(TObject3D::From(kPt3), TObject3D::From(kHorizLine3), obj))
      << "point-line";
  EXPECT_TRUE(mrpt::math::intersect(TObject3D::From(kPt3), TObject3D::From(kSquare3), obj))
      << "point-polygon";
  EXPECT_TRUE(mrpt::math::intersect(TObject3D::From(kPt3), TObject3D::From(kPlaneZ0), obj))
      << "point-plane";
}

TEST(GeometryObjectDispatch3D, SegmentVsAll)
{
  TObject3D obj;
  EXPECT_TRUE(mrpt::math::intersect(TObject3D::From(kHorizSeg3), TObject3D::From(kPt3), obj))
      << "segment-point";
  EXPECT_TRUE(mrpt::math::intersect(TObject3D::From(kHorizSeg3), TObject3D::From(kVertSeg3), obj))
      << "segment-segment";
  EXPECT_TRUE(mrpt::math::intersect(TObject3D::From(kHorizSeg3), TObject3D::From(kVertLine3), obj))
      << "segment-line";
  EXPECT_TRUE(mrpt::math::intersect(TObject3D::From(kHorizSeg3), TObject3D::From(kSquare3), obj))
      << "segment-polygon";
  EXPECT_TRUE(mrpt::math::intersect(TObject3D::From(kCrossSeg3), TObject3D::From(kPlaneZ0), obj))
      << "segment-plane";
}

TEST(GeometryObjectDispatch3D, LineVsAll)
{
  TObject3D obj;
  EXPECT_TRUE(mrpt::math::intersect(TObject3D::From(kHorizLine3), TObject3D::From(kPt3), obj))
      << "line-point";
  EXPECT_TRUE(mrpt::math::intersect(TObject3D::From(kHorizLine3), TObject3D::From(kVertSeg3), obj))
      << "line-segment";
  EXPECT_TRUE(mrpt::math::intersect(TObject3D::From(kHorizLine3), TObject3D::From(kVertLine3), obj))
      << "line-line";
  EXPECT_TRUE(mrpt::math::intersect(TObject3D::From(kHorizLine3), TObject3D::From(kSquare3), obj))
      << "line-polygon";
  // kHorizLine3 lies fully within the Z=0 plane: exercises the "line
  // contained in plane" branch (also a regression check for the bugfix in
  // math::intersect(const TObject3D&, const TObject3D&, TObject3D&) where
  // an uninitialized line was used instead of the actual one).
  ASSERT_TRUE(mrpt::math::intersect(TObject3D::From(kHorizLine3), TObject3D::From(kPlaneZ0), obj))
      << "line-plane";
  EXPECT_TRUE(obj.isLine());
}

TEST(GeometryObjectDispatch3D, PolygonVsAll)
{
  TObject3D obj;
  EXPECT_TRUE(mrpt::math::intersect(TObject3D::From(kSquare3), TObject3D::From(kPt3), obj))
      << "polygon-point";
  EXPECT_TRUE(mrpt::math::intersect(TObject3D::From(kSquare3), TObject3D::From(kVertSeg3), obj))
      << "polygon-segment";
  EXPECT_TRUE(mrpt::math::intersect(TObject3D::From(kSquare3), TObject3D::From(kVertLine3), obj))
      << "polygon-line";
  const TPolygon3D overlapping{
      {1, 1, 0},
      {3, 1, 0},
      {3, 3, 0},
      {1, 3, 0}
  };
  EXPECT_TRUE(mrpt::math::intersect(TObject3D::From(kSquare3), TObject3D::From(overlapping), obj))
      << "polygon-polygon";
  EXPECT_TRUE(mrpt::math::intersect(TObject3D::From(kSquare3), TObject3D::From(kPlaneZ0), obj))
      << "polygon-plane";
}

TEST(GeometryObjectDispatch3D, PlaneVsAll)
{
  TObject3D obj;
  EXPECT_TRUE(mrpt::math::intersect(TObject3D::From(kPlaneZ0), TObject3D::From(kPt3), obj))
      << "plane-point";
  EXPECT_TRUE(mrpt::math::intersect(TObject3D::From(kPlaneZ0), TObject3D::From(kCrossSeg3), obj))
      << "plane-segment";
  EXPECT_TRUE(mrpt::math::intersect(TObject3D::From(kPlaneZ0), TObject3D::From(kHorizLine3), obj))
      << "plane-line";
  const auto samePlaneScaled = TPlane::FromPointAndNormal({1, 1, 0}, {0, 0, 3});
  EXPECT_TRUE(
      mrpt::math::intersect(TObject3D::From(kPlaneZ0), TObject3D::From(samePlaneScaled), obj))
      << "plane-plane";
}

TEST(GeometryObjectDispatch3D, NoIntersectionReturnsFalse)
{
  TObject3D obj;
  const TPoint3D farPt{100, 100, 100};
  EXPECT_FALSE(mrpt::math::intersect(TObject3D::From(kPt3), TObject3D::From(farPt), obj));
}
