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

// Additional geometry.cpp coverage: degenerate cases, throw-paths, and less
// common overloads not exercised by geometry_unittest.cpp.

#include <gtest/gtest.h>
#include <mrpt/math/TLine2D.h>
#include <mrpt/math/TLine3D.h>
#include <mrpt/math/TObject2D.h>
#include <mrpt/math/TObject3D.h>
#include <mrpt/math/TPolygonWithPlane.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/geometry.h>

using namespace mrpt;
using namespace mrpt::math;

// =========================================================================
//  closestFromPointTo{Segment,Line} / squaredDistancePointToLine: degenerate
//  segment (both endpoints equal)
// =========================================================================

TEST(GeometryExtra, ClosestFromPointToSegmentDegenerate)
{
  double ox = 0;
  double oy = 0;
  closestFromPointToSegment(5.0, 5.0, 1.0, 1.0, 1.0, 1.0, ox, oy);
  EXPECT_NEAR(ox, 1.0, 1e-9);
  EXPECT_NEAR(oy, 1.0, 1e-9);
}

TEST(GeometryExtra, ClosestFromPointToLineDegenerate)
{
  double ox = 0;
  double oy = 0;
  closestFromPointToLine(5.0, 5.0, 1.0, 1.0, 1.0, 1.0, ox, oy);
  EXPECT_NEAR(ox, 1.0, 1e-9);
  EXPECT_NEAR(oy, 1.0, 1e-9);
}

TEST(GeometryExtra, SquaredDistancePointToLineDegenerate)
{
  EXPECT_NEAR(squaredDistancePointToLine(4.0, 6.0, 1.0, 2.0, 1.0, 2.0), 9.0 + 16.0, 1e-9);
}

// =========================================================================
//  distance(): TO-DO overloads that must throw
// =========================================================================

TEST(GeometryExtra, DistancePolygonOverloadsThrow)
{
  const TPolygon2D poly2{
      {0, 0},
      {1, 0},
      {0, 1}
  };
  const TPolygon3D poly3{
      {0, 0, 0},
      {1, 0, 0},
      {0, 1, 0}
  };
  const TSegment2D seg2({0, 0}, {1, 0});
  const TSegment3D seg3({0, 0, 0}, {1, 0, 0});
  const TLine2D line2 = TLine2D::FromTwoPoints({0, 0}, {1, 0});
  const TLine3D line3 = TLine3D::FromTwoPoints({0, 0, 0}, {1, 0, 0});
  const TPlane plane = TPlane::FromPointAndNormal({0, 0, 0}, {0, 0, 1});

  EXPECT_THROW(mrpt::math::distance(poly2, poly2), std::exception);
  EXPECT_THROW(mrpt::math::distance(poly2, seg2), std::exception);
  EXPECT_THROW(mrpt::math::distance(poly2, line2), std::exception);
  EXPECT_THROW(mrpt::math::distance(poly3, poly3), std::exception);
  EXPECT_THROW(mrpt::math::distance(poly3, seg3), std::exception);
  EXPECT_THROW(mrpt::math::distance(poly3, line3), std::exception);
  EXPECT_THROW(mrpt::math::distance(poly3, plane), std::exception);
}

// =========================================================================
//  getAngle(): invalid-input throw paths
// =========================================================================

TEST(GeometryExtra, GetAnglePlanePlaneThrowsOnDegeneratePlane)
{
  TPlane degenerate;
  degenerate.coefs[0] = degenerate.coefs[1] = degenerate.coefs[2] = degenerate.coefs[3] = 0.0;
  const auto valid = TPlane::FromPointAndNormal({0, 0, 0}, {0, 0, 1});
  EXPECT_THROW(mrpt::math::getAngle(degenerate, valid), std::exception);
}

TEST(GeometryExtra, GetAnglePlaneLineThrowsOnDegenerateLine)
{
  const auto plane = TPlane::FromPointAndNormal({0, 0, 0}, {0, 0, 1});
  TLine3D degenerate;  // zero director
  EXPECT_THROW(mrpt::math::getAngle(plane, degenerate), std::exception);
}

TEST(GeometryExtra, GetAngleLine3DThrowsOnDegenerateLine)
{
  const auto l = TLine3D::FromTwoPoints({0, 0, 0}, {1, 0, 0});
  TLine3D degenerate;
  EXPECT_THROW(mrpt::math::getAngle(l, degenerate), std::exception);
}

// =========================================================================
//  conformAPlane() / areAligned(): overloads not covered elsewhere
// =========================================================================

TEST(GeometryExtra, ConformAPlaneWithPlaneOutput)
{
  const std::vector<TPoint3D> pts = {
      {0., 0., 0.},
      {1., 0., 0.},
      {1., 1., 0.},
      {0., 1., 0.}
  };
  TPlane pl;
  EXPECT_TRUE(mrpt::math::conformAPlane(pts, pl));
  EXPECT_TRUE(pl.contains({0.5, 0.5, 0.0}));
}

TEST(GeometryExtra, ConformAPlaneTooFewPoints)
{
  const std::vector<TPoint3D> pts = {
      {0., 0., 0.},
      {1., 0., 0.}
  };
  EXPECT_FALSE(mrpt::math::conformAPlane(pts));
}

TEST(GeometryExtra, AreAlignedTooFewPoints2D)
{
  const std::vector<TPoint2D> pts = {
      {0, 0}
  };
  EXPECT_FALSE(mrpt::math::areAligned(pts));
}

TEST(GeometryExtra, AreAlignedTooFewPoints3D)
{
  const std::vector<TPoint3D> pts = {
      {0, 0, 0}
  };
  EXPECT_FALSE(mrpt::math::areAligned(pts));
}

TEST(GeometryExtra, AreAlignedFalse3DWithLineOutput)
{
  const std::vector<TPoint3D> pts = {
      {0, 0,   0},
      {1, 1,   1},
      {2, 2, 3.1}
  };
  TLine3D line;
  EXPECT_FALSE(mrpt::math::areAligned(pts, line));
}

// =========================================================================
//  intersect(): parallel/coincident/contained special cases
// =========================================================================

TEST(GeometryExtra, IntersectLine2DSameLine)
{
  const auto l1 = TLine2D::FromTwoPoints({0, 0}, {1, 0});
  const auto l2 = TLine2D::FromTwoPoints({5, 0}, {10, 0});  // same line, y=0

  TObject2D obj;
  ASSERT_TRUE(mrpt::math::intersect(l1, l2, obj));
  EXPECT_TRUE(obj.isLine());
}

TEST(GeometryExtra, IntersectLine3DSameLine)
{
  const auto l1 = TLine3D::FromTwoPoints({0, 0, 0}, {1, 0, 0});
  const auto l2 = TLine3D::FromTwoPoints({5, 0, 0}, {10, 0, 0});  // same line

  TObject3D obj;
  ASSERT_TRUE(mrpt::math::intersect(l1, l2, obj));
  EXPECT_TRUE(obj.isLine());
}

TEST(GeometryExtra, IntersectLine2DSegmentFullyInside)
{
  // Segment lies exactly on the line -> obj is the segment itself
  const auto l1 = TLine2D::FromTwoPoints({0, 0}, {1, 0});
  const TSegment2D s2(TPoint2D(2, 0), TPoint2D(3, 0));

  TObject2D obj;
  ASSERT_TRUE(mrpt::math::intersect(l1, s2, obj));
  EXPECT_TRUE(obj.isSegment());
}

TEST(GeometryExtra, IntersectSegment2DSegment2DCollinearOverlap)
{
  // Collinear overlapping segments -> intersection is a sub-segment
  const TSegment2D s1(TPoint2D(0, 0), TPoint2D(2, 0));
  const TSegment2D s2(TPoint2D(1, 0), TPoint2D(3, 0));

  TObject2D obj;
  ASSERT_TRUE(mrpt::math::intersect(s1, s2, obj));
  EXPECT_TRUE(obj.isSegment());
}

TEST(GeometryExtra, IntersectSegment2DSegment2DCollinearNoOverlap)
{
  const TSegment2D s1(TPoint2D(0, 0), TPoint2D(1, 0));
  const TSegment2D s2(TPoint2D(2, 0), TPoint2D(3, 0));

  TObject2D obj;
  EXPECT_FALSE(mrpt::math::intersect(s1, s2, obj));
}

TEST(GeometryExtra, IntersectSegment3DSegment3DCollinearOverlap)
{
  const TSegment3D s1(TPoint3D(0, 0, 0), TPoint3D(2, 0, 0));
  const TSegment3D s2(TPoint3D(1, 0, 0), TPoint3D(3, 0, 0));

  TObject3D obj;
  ASSERT_TRUE(mrpt::math::intersect(s1, s2, obj));
  EXPECT_TRUE(obj.isSegment());
}

TEST(GeometryExtra, IntersectSegment3DPlaneFullyInside)
{
  // Segment lies entirely inside the plane Z=0
  const TSegment3D s1(TPoint3D(0, 0, 0), TPoint3D(1, 1, 0));
  const auto plane = TPlane::FromPointAndNormal({0, 0, 0}, {0, 0, 1});

  TObject3D obj;
  ASSERT_TRUE(mrpt::math::intersect(s1, plane, obj));
  EXPECT_TRUE(obj.isSegment());
}

TEST(GeometryExtra, IntersectSegment3DLine3DSameLine)
{
  // Segment's own line matches the given line entirely
  const TSegment3D s1(TPoint3D(0, 0, 0), TPoint3D(1, 0, 0));
  const auto line = TLine3D::FromTwoPoints({5, 0, 0}, {10, 0, 0});

  TObject3D obj;
  ASSERT_TRUE(mrpt::math::intersect(s1, line, obj));
  EXPECT_TRUE(obj.isSegment());
}

TEST(GeometryExtra, IntersectPlaneLine3DContained)
{
  // Line lies entirely inside the plane Z=0
  const auto plane = TPlane::FromPointAndNormal({0, 0, 0}, {0, 0, 1});
  const auto line = TLine3D::FromTwoPoints({0, 0, 0}, {1, 1, 0});

  TObject3D obj;
  ASSERT_TRUE(mrpt::math::intersect(plane, line, obj));
  EXPECT_TRUE(obj.isLine());
}

TEST(GeometryExtra, IntersectPlanePlaneSamePlane)
{
  const auto p1 = TPlane::FromPointAndNormal({0, 0, 0}, {0, 0, 1});
  const auto p2 = TPlane::FromPointAndNormal({1, 1, 0}, {0, 0, 2});  // same plane, scaled normal

  TObject3D obj;
  ASSERT_TRUE(mrpt::math::intersect(p1, p2, obj));
  EXPECT_TRUE(obj.isPlane());
}

// Regression test for a bug where intersect(TObject3D,TObject3D,TObject3D&)
// used a default-constructed (never assigned) TLine3D instead of the actual
// line when the first object is a line and the second a plane.
TEST(GeometryExtra, IntersectObject3D_LineAndPlane_RegressionBug)
{
  const auto line = TLine3D::FromTwoPoints({0, 0, 5}, {1, 0, 5});
  const auto plane = TPlane::FromPointAndNormal({0, 0, 5}, {0, 0, 1});

  const TObject3D o1 = TObject3D::From(line);
  const TObject3D o2 = TObject3D::From(plane);

  TObject3D obj;
  ASSERT_TRUE(mrpt::math::intersect(o1, o2, obj));
  EXPECT_TRUE(obj.isLine());
}

// =========================================================================
//  project3D / project2D — TObject overloads (std::visit dispatch)
// =========================================================================

TEST(GeometryExtra, Project3DObjectPointVariant)
{
  const TObject3D o = TObject3D::From(TPoint3D(1, 0, 0));
  const TPose3D pose{0.0, 0.0, 1.0, 0.0, 0.0, 0.0};

  TObject3D result;
  mrpt::math::project3D(o, pose, result);
  ASSERT_TRUE(result.isPoint());
  const auto p = result.getAs<TPoint3D>();
  EXPECT_NEAR(p.z, 1.0, 1e-9);
}

TEST(GeometryExtra, Project3DObjectSegmentVariant)
{
  const TObject3D o = TObject3D::From(TSegment3D(TPoint3D(0, 0, 0), TPoint3D(1, 0, 0)));
  const TPose3D pose{0.0, 0.0, 1.0, 0.0, 0.0, 0.0};

  TObject3D result;
  mrpt::math::project3D(o, pose, result);
  ASSERT_TRUE(result.isSegment());
}

TEST(GeometryExtra, Project2DObjectPointVariant)
{
  const TObject2D o = TObject2D::From(TPoint2D(1, 0));
  const TPose2D pose{0.0, 1.0, 0.0};

  TObject2D result;
  mrpt::math::project2D(o, pose, result);
  ASSERT_TRUE(result.isPoint());
  const auto p = result.getAs<TPoint2D>();
  EXPECT_NEAR(p.y, 1.0, 1e-9);
}

TEST(GeometryExtra, Project2DObjectSegmentVariant)
{
  const TObject2D o = TObject2D::From(TSegment2D(TPoint2D(0, 0), TPoint2D(1, 0)));
  const TPose2D pose{0.0, 1.0, 0.0};

  TObject2D result;
  mrpt::math::project2D(o, pose, result);
  ASSERT_TRUE(result.isSegment());
}

// =========================================================================
//  intersect(TPolygon2D, TLine2D): vertex-touching (1 point) and
//  non-convex-polygon throw path
// =========================================================================

TEST(GeometryExtra, IntersectPolygon2DLineTangentAtVertex)
{
  // Triangle touching the line y=0 only at the vertex (0,0)
  const TPolygon2D poly = {
      { 0, 0},
      { 1, 1},
      {-1, 1}
  };
  const TLine2D line = TLine2D::FromTwoPoints({-5, 0}, {5, 0});

  TObject2D obj;
  ASSERT_TRUE(mrpt::math::intersect(poly, line, obj));
  EXPECT_TRUE(obj.isPoint());
}

TEST(GeometryExtra, IntersectPolygon2DLineNoIntersection)
{
  const TPolygon2D poly = {
      { 0, 1},
      { 1, 2},
      {-1, 2}
  };
  const TLine2D line = TLine2D::FromTwoPoints({-5, 0}, {5, 0});

  TObject2D obj;
  EXPECT_FALSE(mrpt::math::intersect(poly, line, obj));
}

// =========================================================================
//  intersect(TPolygon3D, ...) overloads
// =========================================================================

TEST(GeometryExtra, IntersectPolygon3DPolygon3D)
{
  // Two triangles on the Z=0 plane that overlap
  const TPolygon3D p1{
      {0, 0, 0},
      {4, 0, 0},
      {0, 4, 0}
  };
  const TPolygon3D p2{
      {1, 1, 0},
      {5, 1, 0},
      {1, 5, 0}
  };

  TObject3D obj;
  EXPECT_TRUE(mrpt::math::intersect(p1, p2, obj));
}

TEST(GeometryExtra, IntersectPolygon3DPolygon3DIncompatibleBounds)
{
  // Disjoint bounding boxes -> quick rejection
  const TPolygon3D p1{
      {0, 0, 0},
      {1, 0, 0},
      {0, 1, 0}
  };
  const TPolygon3D p2{
      {100, 100, 100},
      {101, 100, 100},
      {100, 101, 100}
  };

  TObject3D obj;
  EXPECT_FALSE(mrpt::math::intersect(p1, p2, obj));
}

TEST(GeometryExtra, IntersectPolygon3DPlaneInsidePlane)
{
  // Polygon lies inside the intersecting plane -> whole polygon returned
  const TPolygon3D poly{
      {0, 0, 0},
      {1, 0, 0},
      {0, 1, 0}
  };
  const auto plane = TPlane::FromPointAndNormal({0, 0, 0}, {0, 0, 1});

  TObject3D obj;
  ASSERT_TRUE(mrpt::math::intersect(poly, plane, obj));
  EXPECT_TRUE(obj.isPolygon());
}

TEST(GeometryExtra, IntersectPolygon3DLine3DCoplanar)
{
  // Line lies within the polygon's own plane (Z=0)
  const TPolygon3D poly{
      {0, 0, 0},
      {4, 0, 0},
      {0, 4, 0}
  };
  const auto line = TLine3D::FromTwoPoints({-1, 1, 0}, {3, 1, 0});

  TObject3D obj;
  EXPECT_TRUE(mrpt::math::intersect(poly, line, obj));
}

// =========================================================================
//  getSegmentBisector / getAngleBisector
// =========================================================================

TEST(GeometryExtra, GetSegmentBisector2D)
{
  const TSegment2D seg(TPoint2D(-1, 0), TPoint2D(1, 0));
  TLine2D bis;
  mrpt::math::getSegmentBisector(seg, bis);

  // The perpendicular bisector of a horizontal segment centered at the
  // origin is the vertical line x=0.
  EXPECT_TRUE(bis.contains({0, 0}));
  EXPECT_TRUE(bis.contains({0, 5}));
  EXPECT_FALSE(bis.contains({1, 0}));
}

TEST(GeometryExtra, GetSegmentBisector3D)
{
  const TSegment3D seg(TPoint3D(-1, 0, 0), TPoint3D(1, 0, 0));
  TPlane bis;
  mrpt::math::getSegmentBisector(seg, bis);

  EXPECT_TRUE(bis.contains({0, 0, 0}));
  EXPECT_TRUE(bis.contains({0, 5, 0}));
  EXPECT_TRUE(bis.contains({0, 0, 5}));
  EXPECT_FALSE(bis.contains({1, 0, 0}));
}

TEST(GeometryExtra, GetAngleBisector2DCrossing)
{
  // Two lines crossing at the origin, at +45 and -45 degrees: bisector
  // should be one of the axes.
  const TLine2D l1 = TLine2D::FromTwoPoints({0, 0}, {1, 1});
  const TLine2D l2 = TLine2D::FromTwoPoints({0, 0}, {1, -1});

  TLine2D bis;
  mrpt::math::getAngleBisector(l1, l2, bis);
  EXPECT_TRUE(bis.contains({0, 0}));
}

TEST(GeometryExtra, GetAngleBisector2DParallel)
{
  const TLine2D l1 = TLine2D::FromTwoPoints({0, 0}, {1, 0});
  const TLine2D l2 = TLine2D::FromTwoPoints({0, 2}, {1, 2});

  TLine2D bis;
  mrpt::math::getAngleBisector(l1, l2, bis);
  // Bisector of y=0 and y=2 must be y=1
  EXPECT_TRUE(bis.contains({0, 1}));
  EXPECT_TRUE(bis.contains({5, 1}));
}

TEST(GeometryExtra, GetAngleBisector3D)
{
  const TLine3D l1 = TLine3D::FromTwoPoints({0, 0, 0}, {1, 0, 0});
  const TLine3D l2 = TLine3D::FromTwoPoints({0, 0, 0}, {0, 1, 0});

  TLine3D bis;
  mrpt::math::getAngleBisector(l1, l2, bis);
  EXPECT_TRUE(bis.contains({0, 0, 0}));
}

// =========================================================================
//  traceRay
// =========================================================================

TEST(GeometryExtra, TraceRayHitsPolygon)
{
  // A square polygon at x=5, in the YZ plane, facing the ray direction (+X)
  const TPolygon3D poly{
      {5, -1, -1},
      {5,  1, -1},
      {5,  1,  1},
      {5, -1,  1}
  };
  std::vector<TPolygonWithPlane> planes;
  TPolygonWithPlane::getPlanes({poly}, planes);

  const TPose3D pose{0.0, 0.0, 0.0, 0.0_deg, 0.0_deg, 0.0_deg};  // ray along local +X
  double dist = 0;
  const bool hit = mrpt::math::traceRay(planes, pose, dist);

  EXPECT_TRUE(hit);
  EXPECT_NEAR(dist, 5.0, 1e-3);
}

TEST(GeometryExtra, TraceRayMisses)
{
  const TPolygon3D poly{
      {5, 10, 10},
      {5, 11, 10},
      {5, 11, 11},
      {5, 10, 11}
  };
  std::vector<TPolygonWithPlane> planes;
  TPolygonWithPlane::getPlanes({poly}, planes);

  const TPose3D pose{0.0, 0.0, 0.0, 0.0_deg, 0.0_deg, 0.0_deg};
  double dist = 0;
  const bool hit = mrpt::math::traceRay(planes, pose, dist);

  EXPECT_FALSE(hit);
}

// =========================================================================
//  generateAxisBaseFromDirection / createPlaneFromPoseAndNormal
// =========================================================================

TEST(GeometryExtra, GenerateAxisBaseFromDirectionXAligned)
{
  // Direction is almost aligned with +X, exercising the "any vector in the
  // XY plane" branch (|dx|,|dy| both tiny).
  const auto P = mrpt::math::generateAxisBaseFromDirection(0.0, 0.0, 1.0);
  ASSERT_EQ(P.rows(), 3);
  ASSERT_EQ(P.cols(), 3);
  // First column must be unit +Z
  EXPECT_NEAR(P(2, 0), 1.0, 1e-9);
}

TEST(GeometryExtra, GenerateAxisBaseFromDirectionGeneral)
{
  const auto P = mrpt::math::generateAxisBaseFromDirection(1.0, 2.0, 3.0);
  ASSERT_EQ(P.rows(), 3);
  // Columns must be orthonormal: check column 0 has unit norm
  double n2 = 0;
  for (int i = 0; i < 3; i++)
  {
    n2 += P(i, 0) * P(i, 0);
  }
  EXPECT_NEAR(n2, 1.0, 1e-6);
}

TEST(GeometryExtra, GenerateAxisBaseFromDirectionThrowsOnZero)
{
  EXPECT_THROW(mrpt::math::generateAxisBaseFromDirection(0.0, 0.0, 0.0), std::exception);
}

TEST(GeometryExtra, CreatePlaneFromPoseAndNormal)
{
  const TPose3D pose{0.0, 0.0, 0.0, 0.0_deg, 0.0_deg, 0.0_deg};
  const double normal[3] = {0.0, 0.0, 1.0};
  TPlane plane;
  mrpt::math::createPlaneFromPoseAndNormal(pose, normal, plane);

  EXPECT_TRUE(plane.contains({1, 1, 0}));
  EXPECT_FALSE(plane.contains({0, 0, 1}));
}

// =========================================================================
//  splitInConvexComponents
// =========================================================================

TEST(GeometryExtra, SplitInConvexComponentsTriangleReturnsFalse)
{
  // A triangle (N<=3) is always convex: function returns false (no split).
  const TPolygon2D poly = {
      {0, 0},
      {1, 0},
      {0, 1}
  };
  std::vector<TPolygon2D> components;
  EXPECT_FALSE(mrpt::math::splitInConvexComponents(poly, components));
}

TEST(GeometryExtra, SplitInConvexComponentsConcavePolygon)
{
  // A concave "L-shaped" hexagon:
  const TPolygon2D poly = {
      {0, 0},
      {2, 0},
      {2, 1},
      {1, 1},
      {1, 2},
      {0, 2}
  };
  std::vector<TPolygon2D> components;
  const bool split = mrpt::math::splitInConvexComponents(poly, components);
  EXPECT_TRUE(split);
  EXPECT_GE(components.size(), 2U);
  for (const auto& c : components)
  {
    EXPECT_TRUE(c.isConvex());
  }
}

// =========================================================================
//  assemblePolygons
// =========================================================================

TEST(GeometryExtra, AssemblePolygonsFromSegments)
{
  // Four segments forming a unit square (given out of order and with some
  // reversed direction) should be reassembled into a single closed polygon.
  const std::vector<TSegment3D> segs = {
      TSegment3D(TPoint3D(0, 0, 0), TPoint3D(1, 0, 0)),
      TSegment3D(TPoint3D(1, 1, 0), TPoint3D(1, 0, 0)),  // reversed
      TSegment3D(TPoint3D(1, 1, 0), TPoint3D(0, 1, 0)),
      TSegment3D(TPoint3D(0, 0, 0), TPoint3D(0, 1, 0)),  // reversed
  };

  std::vector<TPolygon3D> polys;
  mrpt::math::assemblePolygons(segs, polys);

  ASSERT_EQ(polys.size(), 1U);
  EXPECT_EQ(polys[0].size(), 4U);
}

TEST(GeometryExtra, AssemblePolygonsWithRemainder)
{
  // A closed square plus one disconnected extra segment (the remainder).
  const std::vector<TSegment3D> segs = {
      TSegment3D(TPoint3D(0, 0, 0), TPoint3D(1, 0, 0)),
      TSegment3D(TPoint3D(1, 0, 0), TPoint3D(1, 1, 0)),
      TSegment3D(TPoint3D(1, 1, 0), TPoint3D(0, 1, 0)),
      TSegment3D(TPoint3D(0, 1, 0), TPoint3D(0, 0, 0)),
      TSegment3D(TPoint3D(10, 10, 10), TPoint3D(11, 10, 10)),
  };

  std::vector<TPolygon3D> polys;
  std::vector<TSegment3D> remainder;
  mrpt::math::assemblePolygons(segs, polys, remainder);

  ASSERT_EQ(polys.size(), 1U);
  EXPECT_EQ(polys[0].size(), 4U);
  EXPECT_EQ(remainder.size(), 1U);
}

TEST(GeometryExtra, AssemblePolygonsFromObjects)
{
  const std::vector<TObject3D> objs = {
      TObject3D::From(TSegment3D(TPoint3D(0, 0, 0), TPoint3D(1, 0, 0))),
      TObject3D::From(TSegment3D(TPoint3D(1, 0, 0), TPoint3D(1, 1, 0))),
      TObject3D::From(TSegment3D(TPoint3D(1, 1, 0), TPoint3D(0, 1, 0))),
      TObject3D::From(TSegment3D(TPoint3D(0, 1, 0), TPoint3D(0, 0, 0))),
      TObject3D::From(TPoint3D(99, 99, 99)),  // not a segment nor polygon: ignored
  };

  std::vector<TPolygon3D> polys;
  mrpt::math::assemblePolygons(objs, polys);

  ASSERT_EQ(polys.size(), 1U);
  EXPECT_EQ(polys[0].size(), 4U);
}

// =========================================================================
//  TPolygonWithPlane intersections
// =========================================================================

TEST(GeometryExtra, TPolygonWithPlaneIntersectSamePlanePolygons)
{
  const TPolygon3D p1{
      {0, 0, 0},
      {4, 0, 0},
      {0, 4, 0}
  };
  const TPolygon3D p2{
      {1, 1, 0},
      {5, 1, 0},
      {1, 5, 0}
  };

  TObject3D obj;
  EXPECT_TRUE(mrpt::math::intersect(p1, p2, obj));
}
