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
#include <mrpt/math/CSparseMatrixTemplate.h>
#include <mrpt/math/TLine2D.h>
#include <mrpt/math/TLine3D.h>
#include <mrpt/math/TObject2D.h>
#include <mrpt/math/TObject3D.h>
#include <mrpt/math/TPolygon2D.h>
#include <mrpt/math/TPolygon3D.h>
#include <mrpt/math/TSegment2D.h>
#include <mrpt/math/TSegment3D.h>
#include <mrpt/math/geometry.h>

#include <vector>

using namespace mrpt::math;

namespace
{
// The unit square in the XY plane, at height z
TPolygon3D squareAtZ(double z)
{
  TPolygon3D p;
  p.emplace_back(0, 0, z);
  p.emplace_back(1, 0, z);
  p.emplace_back(1, 1, z);
  p.emplace_back(0, 1, z);
  return p;
}
}  // namespace

TEST(GeometrySets, getRectangleBounds)
{
  const std::vector<TPoint2D> poly{
      {-1.0,  2.0},
      { 3.0, -4.0},
      { 0.5,  0.5}
  };

  TPoint2D pMin, pMax;
  getRectangleBounds(poly, pMin, pMax);
  EXPECT_NEAR(pMin.x, -1.0, 1e-12);
  EXPECT_NEAR(pMin.y, -4.0, 1e-12);
  EXPECT_NEAR(pMax.x, 3.0, 1e-12);
  EXPECT_NEAR(pMax.y, 2.0, 1e-12);

  const std::vector<TPoint2D> empty;
  EXPECT_THROW(getRectangleBounds(empty, pMin, pMax), std::logic_error);
}

TEST(GeometrySets, getPrismBounds)
{
  const std::vector<TPoint3D> poly{
      {-1.0,  2.0,  5.0},
      { 3.0, -4.0, -2.0},
      { 0.5,  0.5,  0.0}
  };

  TPoint3D pMin, pMax;
  getPrismBounds(poly, pMin, pMax);
  EXPECT_NEAR(pMin.z, -2.0, 1e-12);
  EXPECT_NEAR(pMax.z, 5.0, 1e-12);

  const std::vector<TPoint3D> empty;
  EXPECT_THROW(getPrismBounds(empty, pMin, pMax), std::logic_error);
}

TEST(GeometrySets, intersectPolygonSetsSparse)
{
  // Two identical squares plus one far away: only the coincident pair overlaps
  const std::vector<TPolygon3D> v1{squareAtZ(0), squareAtZ(100)};
  const std::vector<TPolygon3D> v2{squareAtZ(0)};

  CSparseMatrixTemplate<TObject3D> objs;
  const size_t n = intersect(v1, v2, objs);
  EXPECT_EQ(n, 1U);
  EXPECT_EQ(objs.rows(), 2U);
  EXPECT_EQ(objs.cols(), 1U);
}

TEST(GeometrySets, intersectPolygonSetsVector)
{
  const std::vector<TPolygon3D> v1{squareAtZ(0), squareAtZ(100)};
  const std::vector<TPolygon3D> v2{squareAtZ(0)};

  std::vector<TObject3D> objs;
  const size_t n = intersect(v1, v2, objs);
  EXPECT_EQ(n, 1U);
  EXPECT_EQ(objs.size(), 1U);
}

TEST(GeometrySets, intersectGenericVectors)
{
  // The generic template over any pair of intersectable object types:
  const std::vector<TSegment2D> v1{
      TSegment2D({0.0, 0.0}, {10.0, 0.0}), TSegment2D({0.0, 5.0}, {10.0, 5.0})};
  const std::vector<TSegment2D> v2{TSegment2D({5.0, -1.0}, {5.0, 1.0})};

  CSparseMatrixTemplate<TObject2D> sparse;
  EXPECT_EQ(intersect(v1, v2, sparse), 1U);

  std::vector<TObject2D> objs;
  EXPECT_EQ(intersect(v1, v2, objs), 1U);
  ASSERT_EQ(objs.size(), 1U);
  EXPECT_TRUE(objs[0].isPoint());
}

TEST(GeometrySets, assemblePolygonsWithBothRemainders)
{
  // A closed triangle (assembled into a polygon), one dangling segment and one
  // point (neither of which can take part in any polygon):
  std::vector<TObject3D> objs;
  objs.push_back(TObject3D::From(TSegment3D({0, 0, 0}, {1, 0, 0})));
  objs.push_back(TObject3D::From(TSegment3D({1, 0, 0}, {0, 1, 0})));
  objs.push_back(TObject3D::From(TSegment3D({0, 1, 0}, {0, 0, 0})));
  objs.push_back(TObject3D::From(TSegment3D({5, 5, 5}, {6, 5, 5})));
  objs.push_back(TObject3D::From(TPoint3D(9, 9, 9)));

  std::vector<TPolygon3D> polys;
  std::vector<TSegment3D> remainderSegments;
  std::vector<TObject3D> remainderObjects;
  assemblePolygons(objs, polys, remainderSegments, remainderObjects);

  EXPECT_EQ(polys.size(), 1U);
  EXPECT_EQ(remainderSegments.size(), 1U);
  ASSERT_EQ(remainderObjects.size(), 1U);
  EXPECT_TRUE(remainderObjects[0].isPoint());
}

TEST(GeometrySets, distanceBetweenSkewLines)
{
  // Two perpendicular, non-intersecting lines 3 units apart in z
  const TLine3D l1(TPoint3D(0, 0, 0), TPoint3D(1, 0, 0));
  const TLine3D l2(TPoint3D(0, 0, 3), TPoint3D(0, 1, 3));

  EXPECT_NEAR(distance(l1, l2), 3.0, 1e-9);
}

TEST(GeometrySets, splitInConvexComponents3D)
{
  // An L-shaped (concave) polygon lying on the plane z=1
  TPolygon3D poly;
  poly.emplace_back(0, 0, 1);
  poly.emplace_back(2, 0, 1);
  poly.emplace_back(2, 1, 1);
  poly.emplace_back(1, 1, 1);
  poly.emplace_back(1, 2, 1);
  poly.emplace_back(0, 2, 1);

  std::vector<TPolygon3D> comps;
  EXPECT_TRUE(splitInConvexComponents(poly, comps));
  EXPECT_GE(comps.size(), 2U);
  for (const auto& c : comps)
    for (const auto& pt : c) EXPECT_NEAR(pt.z, 1.0, 1e-6);

  // A triangle is already convex, so no split takes place:
  TPolygon3D tri;
  tri.emplace_back(0, 0, 1);
  tri.emplace_back(1, 0, 1);
  tri.emplace_back(0, 1, 1);
  EXPECT_FALSE(splitInConvexComponents(tri, comps));
}

TEST(GeometrySets, intersectPolygonsInPerpendicularPlanes)
{
  // A square in the XY plane and a square in the XZ plane, crossing it:
  TPolygon3D pXY;
  pXY.emplace_back(-1, -1, 0);
  pXY.emplace_back(1, -1, 0);
  pXY.emplace_back(1, 1, 0);
  pXY.emplace_back(-1, 1, 0);

  TPolygon3D pXZ;
  pXZ.emplace_back(-0.5, 0, -1);
  pXZ.emplace_back(0.5, 0, -1);
  pXZ.emplace_back(0.5, 0, 1);
  pXZ.emplace_back(-0.5, 0, 1);

  TObject3D obj;
  ASSERT_TRUE(intersect(pXY, pXZ, obj));

  // The two faces cross along a segment of the line y=z=0:
  TSegment3D s;
  ASSERT_TRUE(obj.getSegment(s));
  EXPECT_NEAR(s.point1.z, 0.0, 1e-9);
  EXPECT_NEAR(s.point2.z, 0.0, 1e-9);
  EXPECT_NEAR(s.point1.y, 0.0, 1e-9);
  EXPECT_NEAR(std::abs(s.point2.x - s.point1.x), 1.0, 1e-9);
}

TEST(GeometrySets, intersectPolygonsInParallelPlanes)
{
  // Parallel but distinct planes never intersect:
  TObject3D obj;
  EXPECT_FALSE(intersect(squareAtZ(0), squareAtZ(1), obj));
}

TEST(GeometrySets, getAngleBisectorParallelOppositeSigns)
{
  // y=0 and y=3, the latter written with a non-unit normal and the opposite
  // orientation. The un-normalized coefficients matter: with A=0 the old,
  // buggy normalization `sqrt(A^2+C^2)` equals |C|, which for C=2 happened to
  // cancel the (also missing) factor of 2 and gave the right answer anyway.
  // C=6 breaks that coincidence, so this pins both halves of the fix.
  const auto l1 = TLine2D::FromCoefficientsABC(0, 1, 0);
  const auto l2 = TLine2D::FromCoefficientsABC(0, -2, 6);

  TLine2D bis;
  getAngleBisector(l1, l2, bis);

  // The bisector must be y=1.5, i.e. equidistant from both:
  EXPECT_NEAR(bis.distance({0.0, 1.5}), 0.0, 1e-9);
  EXPECT_NEAR(bis.distance({5.0, 1.5}), 0.0, 1e-9);
  EXPECT_NEAR(bis.distance({0.0, 0.0}), bis.distance({0.0, 3.0}), 1e-9);
}

TEST(GeometrySets, intersectPolygonSetsInDifferentPlanes)
{
  // A square in the XY plane and one in the XZ plane crossing it: their
  // supporting planes meet along a line, which is the branch of the
  // polygon-vs-polygon intersection that the coplanar case never reaches.
  TPolygon3D pXY;
  pXY.emplace_back(-1, -1, 0);
  pXY.emplace_back(1, -1, 0);
  pXY.emplace_back(1, 1, 0);
  pXY.emplace_back(-1, 1, 0);

  TPolygon3D pXZ;
  pXZ.emplace_back(-0.5, 0, -1);
  pXZ.emplace_back(0.5, 0, -1);
  pXZ.emplace_back(0.5, 0, 1);
  pXZ.emplace_back(-0.5, 0, 1);

  // A third one, parallel to the first but far away, which must not match:
  const TPolygon3D far = squareAtZ(50);

  const std::vector<TPolygon3D> v1{pXY, far};
  const std::vector<TPolygon3D> v2{pXZ};

  CSparseMatrixTemplate<TObject3D> objs;
  EXPECT_EQ(intersect(v1, v2, objs), 1U);
  ASSERT_TRUE(objs.exists(0, 0));

  TSegment3D s;
  ASSERT_TRUE(objs(0, 0).getSegment(s));
  EXPECT_NEAR(std::abs(s.point2.x - s.point1.x), 1.0, 1e-9);

  std::vector<TObject3D> objsVec;
  EXPECT_EQ(intersect(v1, v2, objsVec), 1U);
  ASSERT_EQ(objsVec.size(), 1U);
}

TEST(GeometrySets, intersectPolygon3DWithLine)
{
  TPolygon3D poly;
  poly.emplace_back(-1, -1, 0);
  poly.emplace_back(1, -1, 0);
  poly.emplace_back(1, 1, 0);
  poly.emplace_back(-1, 1, 0);

  TObject3D obj;

  // A line crossing the polygon's plane inside it: a single point
  {
    const TLine3D l(TPoint3D(0, 0, -1), TPoint3D(0, 0, 1));
    ASSERT_TRUE(intersect(poly, l, obj));
    TPoint3D p;
    ASSERT_TRUE(obj.getPoint(p));
    EXPECT_NEAR(p.z, 0.0, 1e-9);
  }
  // A line crossing the plane outside the polygon: no intersection
  {
    const TLine3D l(TPoint3D(10, 10, -1), TPoint3D(10, 10, 1));
    EXPECT_FALSE(intersect(poly, l, obj));
  }
  // A line contained in the polygon's plane and crossing it: a segment
  {
    const TLine3D l(TPoint3D(-5, 0, 0), TPoint3D(5, 0, 0));
    ASSERT_TRUE(intersect(poly, l, obj));
    TSegment3D s;
    ASSERT_TRUE(obj.getSegment(s));
    EXPECT_NEAR(std::abs(s.point2.x - s.point1.x), 2.0, 1e-9);
  }
  // A line parallel to the plane but off it: no intersection
  {
    const TLine3D l(TPoint3D(-5, 0, 1), TPoint3D(5, 0, 1));
    EXPECT_FALSE(intersect(poly, l, obj));
  }
}

TEST(GeometrySets, areAlignedDegenerateInputs)
{
  // Two points are always aligned, and the line through them is returned:
  TLine2D l2;
  EXPECT_TRUE(areAligned(
      std::vector<TPoint2D>{
          {0.0, 0.0},
          {1.0, 1.0}
  },
      l2));
  EXPECT_TRUE(l2.contains({2.0, 2.0}));

  TLine3D l3;
  EXPECT_TRUE(areAligned(
      std::vector<TPoint3D>{
          {0, 0, 0},
          {1, 1, 1}
  },
      l3));
  EXPECT_TRUE(l3.contains({2, 2, 2}));

  // Coincident points do not define a line:
  EXPECT_FALSE(areAligned(
      std::vector<TPoint2D>{
          {1.0, 1.0},
          {1.0, 1.0}
  },
      l2));
  EXPECT_FALSE(areAligned(
      std::vector<TPoint3D>{
          {1, 1, 1},
          {1, 1, 1}
  },
      l3));
}
