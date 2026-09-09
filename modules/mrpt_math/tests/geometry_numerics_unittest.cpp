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

// Degenerate and error branches of the geometry helpers and of the special
// functions in math.cpp / poly_roots.cpp, which the "happy path" tests do not
// reach.

#include <gtest/gtest.h>
#include <mrpt/math/CAtan2LookUpTable.h>
#include <mrpt/math/TLine3D.h>
#include <mrpt/math/TObject2D.h>
#include <mrpt/math/TObject3D.h>
#include <mrpt/math/TPlane.h>
#include <mrpt/math/TPolygon2D.h>
#include <mrpt/math/TPolygon3D.h>
#include <mrpt/math/TSegment2D.h>
#include <mrpt/math/TSegment3D.h>
#include <mrpt/math/distributions.h>
#include <mrpt/math/geometry.h>
#include <mrpt/math/poly_roots.h>
#include <mrpt/math/utils.h>

#include <cmath>
#include <sstream>
#include <stdexcept>

using namespace mrpt::math;

// ---------------------------------------------------------------------------
//  getAngleBisector()
// ---------------------------------------------------------------------------

TEST(GetAngleBisector, ParallelLinesGeneralOrientation)
{
  // Two parallel lines with a non-negligible A coefficient, so the "same sign"
  // test uses coefs[0] rather than coefs[1]:  3x + 4y + C = 0
  const TLine2D l1(3.0, 4.0, 0.0);
  const TLine2D l2(3.0, 4.0, -10.0);  // at distance 2 from l1

  TLine2D bis;
  getAngleBisector(l1, l2, bis);

  // The bisector must be parallel to both and equidistant from them:
  EXPECT_NEAR(bis.coefs[0] * 4.0 - bis.coefs[1] * 3.0, 0.0, 1e-9);
  EXPECT_NEAR(bis.distance(TPoint2D(0, 0)), l1.distance(TPoint2D(0, 0)) + 1.0, 1e-9);
  EXPECT_NEAR(bis.distance(TPoint2D(0, 0)), l2.distance(TPoint2D(0, 0)) - 1.0, 1e-9);
}

TEST(GetAngleBisector, ParallelLinesWithOppositeNormals)
{
  // Same geometric lines as above but the second one written with a flipped
  // normal, which must not move the bisector.
  const TLine2D l1(3.0, 4.0, 0.0);
  const TLine2D l2(-3.0, -4.0, 10.0);

  TLine2D bis;
  getAngleBisector(l1, l2, bis);
  EXPECT_NEAR(bis.distance(TPoint2D(0, 0)), 1.0, 1e-9);
}

TEST(GetAngleBisector, CoincidentLinesReturnTheLineItself)
{
  const TLine2D l1(1.0, 2.0, 3.0);
  TLine2D bis;
  getAngleBisector(l1, l1, bis);

  TLine2D expected = l1;
  expected.unitarize();
  for (size_t i = 0; i < 3; i++)
  {
    EXPECT_NEAR(bis.coefs[i], expected.coefs[i], 1e-12) << "coef " << i;
  }
}

TEST(GetAngleBisector, CrossingLines)
{
  const TLine2D l1(0.0, 1.0, 0.0);  // y = 0
  const TLine2D l2(1.0, 0.0, 0.0);  // x = 0
  TLine2D bis;
  getAngleBisector(l1, l2, bis);
  // Must pass through the origin, at 45 deg:
  EXPECT_NEAR(bis.evaluatePoint(TPoint2D(0, 0)), 0.0, 1e-12);
  EXPECT_NEAR(std::abs(bis.coefs[0]), std::abs(bis.coefs[1]), 1e-12);
}

// ---------------------------------------------------------------------------
//  TLine3D error paths
// ---------------------------------------------------------------------------

TEST(TLine3D, FromTwoIdenticalPointsThrows)
{
  const TPoint3D p(1, 2, 3);
  EXPECT_THROW((void)TLine3D::FromTwoPoints(p, p), std::logic_error);
}

TEST(TLine3D, DistanceToDegenerateLineThrows)
{
  const TLine3D l = TLine3D::FromPointAndDirector({0, 0, 0}, {1, 0, 0});
  TLine3D degenerate = l;
  degenerate.director = {0, 0, 0};

  EXPECT_THROW((void)l.distance(degenerate), std::exception);
  EXPECT_THROW((void)degenerate.distance(l), std::exception);
}

TEST(TLine3D, DistanceReportsMidPointOfTheCommonPerpendicular)
{
  const TLine3D a = TLine3D::FromPointAndDirector({0, 0, 0}, {1, 0, 0});
  const TLine3D b = TLine3D::FromPointAndDirector({0, 0, 2}, {0, 1, 0});

  TPoint3D mid;
  const auto d = a.distance(b, mid);
  ASSERT_TRUE(d.has_value());
  EXPECT_NEAR(*d, 2.0, 1e-9);
  EXPECT_NEAR(mid.z, 1.0, 1e-9);

  // Parallel lines report "no unique common perpendicular":
  const TLine3D parallel = TLine3D::FromPointAndDirector({0, 3, 0}, {1, 0, 0});
  EXPECT_FALSE(a.distance(parallel).has_value());
}

TEST(TLine3D, TextStreaming)
{
  const TLine3D l = TLine3D::FromPointAndDirector({1, 2, 3}, {0, 0, 1});
  std::stringstream ss;
  ss << l;
  EXPECT_FALSE(ss.str().empty());
  EXPECT_EQ(ss.str(), l.asString());
}

// ---------------------------------------------------------------------------
//  assemblePolygons() overloads taking TObject3D
// ---------------------------------------------------------------------------

namespace
{
std::vector<TSegment3D> unitSquareSegments()
{
  const TPoint3D a(0, 0, 0);
  const TPoint3D b(1, 0, 0);
  const TPoint3D c(1, 1, 0);
  const TPoint3D d(0, 1, 0);
  return {TSegment3D(a, b), TSegment3D(b, c), TSegment3D(c, d), TSegment3D(d, a)};
}
}  // namespace

// An empty (or single-segment) input must simply produce nothing, not spin in
// an unsigned-underflowed loop.
TEST(AssemblePolygons, EmptyAndSingleSegmentInputs)
{
  std::vector<TPolygon3D> polys;
  std::vector<TSegment3D> remainder;

  assemblePolygons(std::vector<TSegment3D>{}, polys, remainder);
  EXPECT_TRUE(polys.empty());
  EXPECT_TRUE(remainder.empty());

  assemblePolygons(std::vector<TSegment3D>{TSegment3D({0, 0, 0}, {1, 0, 0})}, polys, remainder);
  EXPECT_TRUE(polys.empty());
  EXPECT_EQ(remainder.size(), 1U);

  // Zero-length segments go straight to the remainder:
  polys.clear();
  remainder.clear();
  assemblePolygons(std::vector<TSegment3D>{TSegment3D({1, 1, 1}, {1, 1, 1})}, polys, remainder);
  EXPECT_TRUE(polys.empty());
  EXPECT_EQ(remainder.size(), 1U);
}

TEST(AssemblePolygons, FromObjectsKeepsExistingPolygons)
{
  std::vector<TObject3D> objs;
  for (const auto& s : unitSquareSegments())
  {
    objs.emplace_back(TObject3D::From(s));
  }
  const TPolygon3D triangle({
      {0, 0, 5},
      {1, 0, 5},
      {0, 1, 5}
  });
  objs.emplace_back(TObject3D::From(triangle));

  std::vector<TPolygon3D> polys;
  assemblePolygons(objs, polys);
  // The pre-existing polygon plus the square assembled from the 4 segments:
  EXPECT_EQ(polys.size(), 2U);
}

TEST(AssemblePolygons, FromObjectsReportsUnusedObjects)
{
  std::vector<TObject3D> objs;
  for (const auto& s : unitSquareSegments())
  {
    objs.emplace_back(TObject3D::From(s));
  }
  // A lone segment that closes nothing, plus a point:
  objs.emplace_back(TObject3D::From(TSegment3D({5, 5, 5}, {6, 6, 6})));
  objs.emplace_back(TObject3D::From(TPoint3D(9, 9, 9)));

  std::vector<TPolygon3D> polys;
  std::vector<TObject3D> remainder;
  assemblePolygons(objs, polys, remainder);

  EXPECT_EQ(polys.size(), 1U);
  EXPECT_EQ(remainder.size(), 2U);
}

TEST(AssemblePolygons, FromObjectsWithSplitRemainders)
{
  std::vector<TObject3D> objs;
  for (const auto& s : unitSquareSegments())
  {
    objs.emplace_back(TObject3D::From(s));
  }
  objs.emplace_back(TObject3D::From(TSegment3D({5, 5, 5}, {6, 6, 6})));
  objs.emplace_back(TObject3D::From(TPoint3D(9, 9, 9)));

  std::vector<TPolygon3D> polys;
  std::vector<TSegment3D> leftoverSegments;
  std::vector<TObject3D> leftoverObjects;
  assemblePolygons(objs, polys, leftoverSegments, leftoverObjects);

  EXPECT_EQ(polys.size(), 1U);
  EXPECT_EQ(leftoverSegments.size(), 1U);
  EXPECT_EQ(leftoverObjects.size(), 1U);
}

// ---------------------------------------------------------------------------
//  splitInConvexComponents()
// ---------------------------------------------------------------------------

TEST(SplitInConvexComponents, ConcavePolygonIsSplit)
{
  // An "L"-shaped (concave) polygon:
  const TPolygon2D poly({
      {0, 0},
      {3, 0},
      {3, 1},
      {1, 1},
      {1, 3},
      {0, 3}
  });

  std::vector<TPolygon2D> comps;
  ASSERT_TRUE(splitInConvexComponents(poly, comps));
  EXPECT_GE(comps.size(), 2U);
  for (const auto& c : comps)
  {
    EXPECT_TRUE(c.isConvex());
  }
}

TEST(SplitInConvexComponents, ConvexPolygonIsReturnedAsIs)
{
  const TPolygon2D square({
      {0, 0},
      {1, 0},
      {1, 1},
      {0, 1}
  });
  std::vector<TPolygon2D> comps;
  EXPECT_FALSE(splitInConvexComponents(square, comps));
}

TEST(SplitInConvexComponents, DegeneratePolygonIsRejected)
{
  const TPolygon2D twoPoints({
      {0, 0},
      {1, 1}
  });
  std::vector<TPolygon2D> comps;
  EXPECT_FALSE(splitInConvexComponents(twoPoints, comps));
}

// ---------------------------------------------------------------------------
//  math.cpp special functions
// ---------------------------------------------------------------------------

TEST(MathSpecialFunctions, Factorial64Overflows)
{
  EXPECT_EQ(factorial64(0), 1ULL);
  EXPECT_EQ(factorial64(20), 2432902008176640000ULL);
  EXPECT_THROW((void)factorial64(21), std::overflow_error);

  // The floating-point version has no such limit:
  EXPECT_GT(factorial(25), 0.0);
}

TEST(MathSpecialFunctions, NormalCdfFarTails)
{
  // |x| large enough to take the asymptotic-expansion branch of erfc():
  EXPECT_NEAR(normalCDF(-10.0), 0.0, 1e-12);
  EXPECT_NEAR(normalCDF(10.0), 1.0, 1e-12);
  EXPECT_NEAR(normalCDF(-40.0), 0.0, 1e-12);
  EXPECT_NEAR(normalCDF(40.0), 1.0, 1e-12);
  // Monotonic and symmetric around 0:
  EXPECT_NEAR(normalCDF(0.0), 0.5, 1e-12);
  EXPECT_LT(normalCDF(-3.0), normalCDF(-2.0));
}

TEST(MathSpecialFunctions, NoncentralChi2LargeArguments)
{
  // A large non-centrality parameter drives the series far enough to exercise
  // the log-domain accumulation branch.
  const double p = noncentralChi2CDF(4, 50.0, 60.0);
  EXPECT_GE(p, 0.0);
  EXPECT_LE(p, 1.0);

  // With zero non-centrality it must agree with the central chi2 CDF:
  EXPECT_NEAR(noncentralChi2CDF(3, 0.0, 3.0), chi2CDF(3, 3.0), 1e-6);
}

// ---------------------------------------------------------------------------
//  poly_roots
// ---------------------------------------------------------------------------

TEST(PolyRoots, QuarticWithComplexIntermediateRoots)
{
  // (x^2+1)(x^2+4) = x^4 + 5x^2 + 4 : no real roots, but the solver's
  // intermediate resolvent has complex square roots.
  double r[4];
  const int n = solve_poly4(r, 0.0, 5.0, 0.0, 4.0);
  EXPECT_EQ(n, 0);
}

TEST(PolyRoots, QuarticWithFourRealRoots)
{
  // (x-1)(x+1)(x-2)(x+2) = x^4 -5x^2 +4
  double r[4];
  const int n = solve_poly4(r, 0.0, -5.0, 0.0, 4.0);
  ASSERT_EQ(n, 4);
  std::sort(r, r + 4);
  EXPECT_NEAR(r[0], -2.0, 1e-9);
  EXPECT_NEAR(r[1], -1.0, 1e-9);
  EXPECT_NEAR(r[2], 1.0, 1e-9);
  EXPECT_NEAR(r[3], 2.0, 1e-9);
}

TEST(PolyRoots, QuadraticRootsAreSorted)
{
  double x1 = 0;
  double x2 = 0;
  // x^2 - x - 6 = (x-3)(x+2)
  ASSERT_EQ(solve_poly2(1.0, -1.0, -6.0, x1, x2), 2);
  EXPECT_LT(x1, x2);
  EXPECT_NEAR(x1, -2.0, 1e-9);
  EXPECT_NEAR(x2, 3.0, 1e-9);
}

// ---------------------------------------------------------------------------
//  CAtan2LookUpTable
// ---------------------------------------------------------------------------

TEST(CAtan2LookUpTable, DefaultConstructedCoversTheUnitSquare)
{
  const CAtan2LookUpTable lut;  // default: [-1,1]x[-1,1], 0.5 m resolution
  double atan2val = 0;
  EXPECT_TRUE(lut.atan2(0.5, 0.5, atan2val));
  EXPECT_NEAR(atan2val, std::atan2(0.5, 0.5), 0.5);

  // Outside the table:
  EXPECT_FALSE(lut.atan2(10.0, 10.0, atan2val));
}

TEST(CAtan2LookUpTable, ResizeWithTheSameResolutionKeepsTheGrid)
{
  CAtan2LookUpTable lut(-1.0, 1.0, -1.0, 1.0, 0.25);
  double v = 0;
  ASSERT_TRUE(lut.atan2(0.5, 0.5, v));

  // Same resolution => the grid is resized in place, not rebuilt:
  lut.resize(-2.0, 2.0, -2.0, 2.0, 0.25);
  EXPECT_TRUE(lut.atan2(1.5, 1.5, v));
  EXPECT_NEAR(v, std::atan2(1.5, 1.5), 0.3);

  // Different resolution => full rebuild:
  lut.resize(-2.0, 2.0, -2.0, 2.0, 0.5);
  EXPECT_TRUE(lut.atan2(-1.5, 1.5, v));
  EXPECT_NEAR(v, std::atan2(-1.5, 1.5), 0.5);
}

// ---------------------------------------------------------------------------
//  TSegment3D: closest-point corner cases
// ---------------------------------------------------------------------------

TEST(TSegment3D, DistanceBetweenSkewSegmentsClampsAtTheEndpoints)
{
  // Two skew segments whose closest points are both endpoints, which drives
  // the clamping branches of the segment-segment distance.
  const TSegment3D a({0, 0, 0}, {1, 0, 0});
  const TSegment3D b({5, 1, 1}, {6, 1, 1});
  EXPECT_NEAR(a.distance(b), std::sqrt(16.0 + 1.0 + 1.0), 1e-9);

  const TSegment3D c({-5, 1, 1}, {-4, 1, 1});
  EXPECT_NEAR(a.distance(c), std::sqrt(16.0 + 1.0 + 1.0), 1e-9);

  // Antiparallel, overlapping projections:
  const TSegment3D d({1, 2, 0}, {0, 2, 0});
  EXPECT_NEAR(a.distance(d), 2.0, 1e-9);

  // Intersecting segments:
  const TSegment3D e({0.5, -1, 0}, {0.5, 1, 0});
  EXPECT_NEAR(a.distance(e), 0.0, 1e-9);
}

TEST(TSegment3D, DegenerateSegmentBehavesLikeAPoint)
{
  const TSegment3D pointLike({2, 2, 2}, {2, 2, 2});
  const TSegment3D a({0, 0, 0}, {1, 0, 0});
  EXPECT_NEAR(a.distance(pointLike), std::sqrt(1.0 + 4.0 + 4.0), 1e-9);
}

// ---------------------------------------------------------------------------
//  intersect() branches for polygons against planes / lines
// ---------------------------------------------------------------------------

TEST(GeometryIntersect, PolygonAgainstACuttingPlane)
{
  // A square in the z=0 plane, cut by the x=0 plane:
  const TPolygon3D square({
      {-1, -1, 0},
      { 1, -1, 0},
      { 1,  1, 0},
      {-1,  1, 0}
  });
  const TPlane x0(1, 0, 0, 0);  // x = 0

  TObject3D obj;
  ASSERT_TRUE(intersect(square, x0, obj));
  // The intersection is the segment from (0,-1,0) to (0,1,0):
  TSegment3D s;
  ASSERT_TRUE(obj.getSegment(s));
  EXPECT_NEAR(s.length(), 2.0, 1e-9);
  EXPECT_NEAR(s.point1.x, 0.0, 1e-9);
  EXPECT_NEAR(s.point2.x, 0.0, 1e-9);
}

TEST(GeometryIntersect, PolygonCoplanarWithThePlane)
{
  const TPolygon3D square({
      {0, 0, 0},
      {1, 0, 0},
      {1, 1, 0},
      {0, 1, 0}
  });
  const TPlane z0(0, 0, 1, 0);  // z = 0

  TObject3D obj;
  ASSERT_TRUE(intersect(square, z0, obj));
  EXPECT_TRUE(obj.isPolygon());
}

TEST(GeometryIntersect, PolygonMissingThePlaneEntirely)
{
  const TPolygon3D square({
      {0, 0, 0},
      {1, 0, 0},
      {1, 1, 0},
      {0, 1, 0}
  });
  const TPlane z5(0, 0, 1, -5);  // z = 5, parallel and disjoint

  TObject3D obj;
  EXPECT_FALSE(intersect(square, z5, obj));
}

TEST(GeometryIntersect, LineCollinearWithASegment)
{
  // A line that contains the segment yields the segment itself:
  const TSegment2D seg({0, 0}, {2, 0});
  const TLine2D onIt(seg);

  TObject2D obj;
  ASSERT_TRUE(intersect(onIt, seg, obj));
  TSegment2D got;
  ASSERT_TRUE(obj.getSegment(got));
  EXPECT_NEAR(got.length(), 2.0, 1e-9);

  // A crossing line yields the crossing point...
  const TLine2D crossing(1.0, 0.0, -1.0);  // x = 1
  ASSERT_TRUE(intersect(crossing, seg, obj));
  TPoint2D p;
  ASSERT_TRUE(obj.getPoint(p));
  EXPECT_NEAR(p.x, 1.0, 1e-9);

  // ... but only if the crossing falls inside the segment.
  const TLine2D beyond(1.0, 0.0, -5.0);  // x = 5
  EXPECT_FALSE(intersect(beyond, seg, obj));
}
