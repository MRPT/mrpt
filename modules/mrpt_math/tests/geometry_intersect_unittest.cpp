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
#include <mrpt/math/TLine2D.h>
#include <mrpt/math/TLine3D.h>
#include <mrpt/math/TObject2D.h>
#include <mrpt/math/TObject3D.h>
#include <mrpt/math/TPlane.h>
#include <mrpt/math/TPolygon2D.h>
#include <mrpt/math/TPolygon3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/TSegment2D.h>
#include <mrpt/math/TSegment3D.h>
#include <mrpt/math/geometry.h>
#include <mrpt/random/RandomGenerators.h>

#include <algorithm>
#include <cmath>
#include <vector>

using namespace mrpt::math;

namespace
{
mrpt::random::CRandomGenerator& rng() { return mrpt::random::getRandomGenerator(); }

double cross2(const TPoint2D& a, const TPoint2D& b) { return a.x * b.y - a.y * b.x; }

TVector3D randomUnitVector()
{
  for (;;)
  {
    const TVector3D v(rng().drawUniform(-1, 1), rng().drawUniform(-1, 1), rng().drawUniform(-1, 1));
    const double n = v.norm();
    if (n > 0.1 && n <= 1.0)
    {
      return v * (1.0 / n);
    }
  }
}

TPose3D randomPose()
{
  return {rng().drawUniform(-5, 5), rng().drawUniform(-5, 5),     rng().drawUniform(-5, 5),
          rng().drawUniform(-3, 3), rng().drawUniform(-1.5, 1.5), rng().drawUniform(-3, 3)};
}

// Independent parametric intersection of two non-parallel segments.
bool refSegmentCross(const TSegment2D& s1, const TSegment2D& s2, TPoint2D& out, double& margin)
{
  const TPoint2D r = s1.point2 - s1.point1;
  const TPoint2D s = s2.point2 - s2.point1;
  const double den = cross2(r, s);
  const TPoint2D qp = s2.point1 - s1.point1;
  const double t = cross2(qp, s) / den;
  const double u = cross2(qp, r) / den;
  margin = std::min({std::abs(t), std::abs(1 - t), std::abs(u), std::abs(1 - u)});
  out = s1.point1 + r * t;
  return t >= 0 && t <= 1 && u >= 0 && u <= 1;
}

// Separating axis test for two convex quadrilaterals. Returns the signed
// separation along the best axis (positive: disjoint, negative: overlap).
double satSeparation(const std::vector<TPoint2D>& a, const std::vector<TPoint2D>& b)
{
  double best = -1e300;
  for (const auto* poly : {&a, &b})
  {
    const size_t n = poly->size();
    for (size_t i = 0; i < n; i++)
    {
      const TPoint2D e = (*poly)[(i + 1) % n] - (*poly)[i];
      const TPoint2D axis(-e.y, e.x);
      double aMin = 1e300;
      double aMax = -1e300;
      double bMin = 1e300;
      double bMax = -1e300;
      for (const auto& p : a)
      {
        const double d = (p.x * axis.x + p.y * axis.y) / axis.norm();
        aMin = std::min(aMin, d);
        aMax = std::max(aMax, d);
      }
      for (const auto& p : b)
      {
        const double d = (p.x * axis.x + p.y * axis.y) / axis.norm();
        bMin = std::min(bMin, d);
        bMax = std::max(bMax, d);
      }
      best = std::max(best, std::max(bMin - aMax, aMin - bMax));
    }
  }
  return best;
}

double overlap1D(double a0, double a1, double b0, double b1)
{
  return std::max(0.0, std::min(a1, b1) - std::max(a0, b0));
}

TPolygon2D axisAlignedRect(double x0, double y0, double x1, double y1, bool ccw)
{
  TPolygon2D p{
      {x0, y0},
      {x1, y0},
      {x1, y1},
      {x0, y1}
  };
  if (!ccw)
  {
    std::reverse(p.begin(), p.end());
  }
  return p;
}
}  // namespace

TEST(GeometryIntersect, Segment2DRandomCrossingsAgreeWithParametric)
{
  rng().randomize(1234);
  int checked = 0;
  for (int k = 0; k < 3000; k++)
  {
    const TPoint2D a(rng().drawUniform(-5, 5), rng().drawUniform(-5, 5));
    const TPoint2D b(rng().drawUniform(-5, 5), rng().drawUniform(-5, 5));
    const TPoint2D c(rng().drawUniform(-5, 5), rng().drawUniform(-5, 5));
    const TPoint2D d(rng().drawUniform(-5, 5), rng().drawUniform(-5, 5));
    const TSegment2D s1(a, b);
    const TSegment2D s2(c, d);
    if (s1.length() < 1.0 || s2.length() < 1.0)
    {
      continue;
    }
    const double sinAng = std::abs(cross2(b - a, d - c)) / (s1.length() * s2.length());
    if (sinAng < 0.05)
    {
      continue;
    }
    TPoint2D pRef;
    double margin = 0;
    const bool crossRef = refSegmentCross(s1, s2, pRef, margin);
    if (margin < 1e-3)
    {
      continue;
    }
    TObject2D obj;
    const bool cross = intersect(s1, s2, obj);
    ASSERT_EQ(cross, crossRef) << s1 << " " << s2;
    if (cross)
    {
      ASSERT_TRUE(obj.isPoint());
      EXPECT_NEAR((obj.getAs<TPoint2D>() - pRef).norm(), 0.0, 1e-9);
    }
    checked++;
  }
  EXPECT_GT(checked, 1000);
}

TEST(GeometryIntersect, ShortCrossingSegments2D)
{
  // The result must not depend on the segments' scale.
  for (const double scale : {1.0, 1e-2, 1e-3})
  {
    const TSegment2D s1({0, 0}, {scale, scale});
    const TSegment2D s2({0, scale}, {scale, 0});
    TObject2D obj;
    ASSERT_TRUE(intersect(s1, s2, obj)) << "scale=" << scale;
    ASSERT_TRUE(obj.isPoint()) << "scale=" << scale << " got " << obj.asString();
    EXPECT_NEAR(obj.getAs<TPoint2D>().x, 0.5 * scale, 1e-9 * scale);
    EXPECT_NEAR(obj.getAs<TPoint2D>().y, 0.5 * scale, 1e-9 * scale);
  }
}

TEST(GeometryIntersect, ShortSegments3D)
{
  // The result must not depend on the segments' scale.
  const TSegment3D s1({0, 0, 0}, {1e-3, 1e-3, 0});
  const TSegment3D s2({0, 1e-3, 0}, {1e-3, 0, 0});
  TObject3D obj;
  ASSERT_TRUE(intersect(s1, s2, obj));
  ASSERT_TRUE(obj.isPoint()) << obj.asString();
  EXPECT_NEAR((obj.getAs<TPoint3D>() - TPoint3D(5e-4, 5e-4, 0)).norm(), 0.0, 1e-12);

  // Same segments, the second one lifted 1 mm along +Z: they do not touch.
  const TSegment3D s3({0, 1e-3, 1e-3}, {1e-3, 0, 1e-3});
  EXPECT_FALSE(intersect(s1, s3, obj)) << obj.asString();
}

TEST(GeometryIntersect, CollinearOverlapNearlyAxisAligned)
{
  // Collinear segments almost aligned with a coordinate axis.
  {
    const TSegment2D s1({0, 0}, {20, 2e-5});
    const TSegment2D s2({15, 1.5e-5}, {40, 4e-5});
    TObject2D obj;
    ASSERT_TRUE(intersect(s1, s2, obj));
    ASSERT_TRUE(obj.isSegment()) << obj.asString();
    EXPECT_NEAR(obj.getAs<TSegment2D>().length(), 5.0, 1e-6);
  }
  {
    const TSegment3D s1({0, 0, 0}, {2e-5, 20, 0});
    const TSegment3D s2({1.5e-5, 15, 0}, {4e-5, 40, 0});
    TObject3D obj;
    ASSERT_TRUE(intersect(s1, s2, obj));
    ASSERT_TRUE(obj.isSegment()) << obj.asString();
    EXPECT_NEAR(obj.getAs<TSegment3D>().length(), 5.0, 1e-6);
  }
}

TEST(GeometryIntersect, PolygonVsVerticalLineWithSmallCoefficients)
{
  // Vertical line with small (unnormalized) coefficients.
  const TPolygon2D sq{
      {0, 0},
      {1, 0},
      {1, 1},
      {0, 1}
  };
  const TLine2D lin(1e-6, 0, -0.5e-6);  // the line x=0.5
  TObject2D obj;
  ASSERT_TRUE(intersect(sq, lin, obj));
  ASSERT_TRUE(obj.isSegment()) << obj.asString();
  const auto s = obj.getAs<TSegment2D>();
  EXPECT_NEAR(s.point1.x, 0.5, 1e-9);
  EXPECT_NEAR(s.point2.x, 0.5, 1e-9);
  EXPECT_NEAR(s.length(), 1.0, 1e-9);

  // A short vertical segment fully inside the polygon must intersect it.
  EXPECT_TRUE(intersect(sq, TSegment2D({0.5, 0.5}, {0.5, 0.500005}), obj));
}

TEST(GeometryIntersect, SutherlandHodgmanRectanglesMatchAnalyticArea)
{
  rng().randomize(1234);
  for (int k = 0; k < 400; k++)
  {
    double ax0 = rng().drawUniform(-5, 5);
    double ax1 = rng().drawUniform(-5, 5);
    double ay0 = rng().drawUniform(-5, 5);
    double ay1 = rng().drawUniform(-5, 5);
    double bx0 = rng().drawUniform(-5, 5);
    double bx1 = rng().drawUniform(-5, 5);
    double by0 = rng().drawUniform(-5, 5);
    double by1 = rng().drawUniform(-5, 5);
    if (ax0 > ax1)
    {
      std::swap(ax0, ax1);
    }
    if (ay0 > ay1)
    {
      std::swap(ay0, ay1);
    }
    if (bx0 > bx1)
    {
      std::swap(bx0, bx1);
    }
    if (by0 > by1)
    {
      std::swap(by0, by1);
    }
    const double expected = overlap1D(ax0, ax1, bx0, bx1) * overlap1D(ay0, ay1, by0, by1);
    for (const bool subjCCW : {true, false})
    {
      for (const bool clipCCW : {true, false})
      {
        const auto subj = axisAlignedRect(ax0, ay0, ax1, ay1, subjCCW);
        const auto clip = axisAlignedRect(bx0, by0, bx1, by1, clipCCW);
        const auto res = intersect(subj, clip);
        const double area = std::abs(signedArea(res));
        EXPECT_NEAR(area, expected, 1e-9);
      }
    }
  }
}

TEST(GeometryIntersect, SutherlandHodgmanSquareAndDiamond)
{
  const TPolygon2D sq{
      {0, 0},
      {2, 0},
      {2, 2},
      {0, 2}
  };
  const TPolygon2D diamond{
      { 2.5,  1.0},
      { 1.0,  2.5},
      {-0.5,  1.0},
      { 1.0, -0.5}
  };
  // Diamond area 4.5 minus four corner triangles of area 0.25 each:
  EXPECT_NEAR(std::abs(signedArea(intersect(sq, diamond))), 3.5, 1e-12);
  EXPECT_NEAR(std::abs(signedArea(intersect(diamond, sq))), 3.5, 1e-12);
}

TEST(GeometryIntersect, SutherlandHodgmanSubjectVertexOnClipEdge)
{
  // Subject vertices lying exactly on a clipping edge, for both windings.
  const TPolygon2D clipCCW{
      {0, 0},
      {1, 0},
      {1, 1},
      {0, 1}
  };
  const TPolygon2D clipCW{
      {0, 0},
      {0, 1},
      {1, 1},
      {1, 0}
  };
  const TPolygon2D touchFromInside{
      {0.5, 0.0},
      {0.8, 0.5},
      {0.2, 0.5}
  };
  const TPolygon2D crossing{
      {-0.5, 0.5},
      {   0, 0.8},
      { 0.5, 0.5},
      {   0, 0.2}
  };
  for (const auto* clip : {&clipCCW, &clipCW})
  {
    for (const auto* subj : {&touchFromInside, &crossing})
    {
      TPolygon2D res;
      ASSERT_NO_THROW(res = intersect(*subj, *clip));
      EXPECT_NEAR(std::abs(signedArea(res)), 0.15, 1e-12);
    }
  }
}

TEST(GeometryIntersect, SutherlandHodgmanNearlyCollinearEdge)
{
  // A subject edge almost collinear with a clipping edge.
  const TPolygon2D clip{
      {100, 100},
      {101, 100},
      {101, 101},
      {100, 101}
  };
  const TPolygon2D subj{
      {100.5, 100 - 2e-8},
      {100.7, 100 + 2e-8},
      {100.7,      100.5},
      {100.5,      100.5}
  };
  const auto res = intersect(subj, clip);
  EXPECT_NEAR(std::abs(signedArea(res)), 0.1, 1e-6);
  for (const auto& v : res)
  {
    EXPECT_GT(v.x, 100 - 1e-6) << v;
    EXPECT_GT(v.y, 100 - 1e-6) << v;
  }
}

TEST(GeometryIntersect, RectanglesIntersectionAgreesWithSAT)
{
  rng().randomize(1234);
  int nHit = 0;
  int nMiss = 0;
  for (int k = 0; k < 3000; k++)
  {
    const double w1 = rng().drawUniform(0.2, 3);
    const double h1 = rng().drawUniform(0.2, 3);
    const double x2min = rng().drawUniform(-2, 0);
    const double y2min = rng().drawUniform(-2, 0);
    const double x2max = x2min + rng().drawUniform(0.2, 3);
    const double y2max = y2min + rng().drawUniform(0.2, 3);
    const double px = rng().drawUniform(-4, 4);
    const double py = rng().drawUniform(-4, 4);
    const double phi = rng().drawUniform(-M_PI, M_PI);

    const std::vector<TPoint2D> r1 = {
        { 0,  0},
        {w1,  0},
        {w1, h1},
        { 0, h1}
    };
    std::vector<TPoint2D> r2;
    for (const auto& p : std::vector<TPoint2D>{
             {x2min, y2min},
             {x2max, y2min},
             {x2max, y2max},
             {x2min, y2max}
    })
    {
      r2.push_back(TPose2D(px, py, phi).composePoint(p));
    }
    const double sep = satSeparation(r1, r2);
    if (std::abs(sep) < 1e-6)
    {
      continue;  // touching configurations are tolerance-dependent
    }
    const bool expected = sep < 0;
    const bool got = RectanglesIntersection(0, w1, 0, h1, x2min, x2max, y2min, y2max, px, py, phi);
    ASSERT_EQ(got, expected) << "k=" << k << " sep=" << sep;
    if (expected)
    {
      nHit++;
    }
    else
    {
      nMiss++;
    }
  }
  EXPECT_GT(nHit, 300);
  EXPECT_GT(nMiss, 300);
}

TEST(GeometryIntersect, PlanePlaneRandomLineLiesOnBoth)
{
  rng().randomize(1234);
  for (int k = 0; k < 500; k++)
  {
    const TVector3D n1 = randomUnitVector() * rng().drawUniform(0.1, 10);
    const TVector3D n2 = randomUnitVector() * rng().drawUniform(0.1, 10);
    if ((n1.unitarize().cross(n2.unitarize())).norm() < 0.05)
    {
      continue;
    }
    const TPlane p1(n1.x, n1.y, n1.z, rng().drawUniform(-10, 10));
    const TPlane p2(n2.x, n2.y, n2.z, rng().drawUniform(-10, 10));
    TObject3D obj;
    ASSERT_TRUE(intersect(p1, p2, obj));
    ASSERT_TRUE(obj.isLine());
    const auto lin = obj.getAs<TLine3D>();
    for (const double t : {0.0, 1.0, -7.5})
    {
      const TPoint3D q = lin.pBase + lin.director * t;
      EXPECT_NEAR(p1.distance(q), 0.0, 1e-8);
      EXPECT_NEAR(p2.distance(q), 0.0, 1e-8);
    }
  }
}

TEST(GeometryIntersect, PlaneLineRandomPointLiesOnBoth)
{
  rng().randomize(1234);
  for (int k = 0; k < 500; k++)
  {
    const TVector3D n = randomUnitVector();
    const TPoint3D onPlane(
        rng().drawUniform(-5, 5), rng().drawUniform(-5, 5), rng().drawUniform(-5, 5));
    const TPlane pl = TPlane::FromPointAndNormal(onPlane, n);
    const TVector3D dir = randomUnitVector() * rng().drawUniform(0.5, 5);
    if (std::abs(dir.unitarize().dot(n)) < 0.05)
    {
      continue;
    }
    const double t = rng().drawUniform(-3, 3);
    const TLine3D lin = TLine3D::FromPointAndDirector(onPlane + dir * t, dir);
    TObject3D obj;
    ASSERT_TRUE(intersect(pl, lin, obj));
    ASSERT_TRUE(obj.isPoint());
    EXPECT_NEAR((obj.getAs<TPoint3D>() - onPlane).norm(), 0.0, 1e-9);
  }
}

TEST(GeometryIntersect, Line3DRandomCrossingsRecoverThePoint)
{
  rng().randomize(1234);
  for (int k = 0; k < 2000; k++)
  {
    const TPoint3D P(
        rng().drawUniform(-10, 10), rng().drawUniform(-10, 10), rng().drawUniform(-10, 10));
    const TVector3D d1 = randomUnitVector() * rng().drawUniform(0.5, 50);
    const TVector3D d2 = randomUnitVector() * rng().drawUniform(0.5, 50);
    if (d1.unitarize().cross(d2.unitarize()).norm() < 0.05)
    {
      continue;
    }
    const TLine3D l1 = TLine3D::FromPointAndDirector(P + d1 * rng().drawUniform(-2, 2), d1);
    const TLine3D l2 = TLine3D::FromPointAndDirector(P + d2 * rng().drawUniform(-2, 2), d2);
    TObject3D obj;
    ASSERT_TRUE(intersect(l1, l2, obj));
    ASSERT_TRUE(obj.isPoint());
    EXPECT_NEAR((obj.getAs<TPoint3D>() - P).norm(), 0.0, 1e-8);

    // Shift the second line off by 0.1 along the common normal: skew lines.
    const TVector3D nrm = d1.cross(d2).unitarize();
    const TLine3D l3 = TLine3D::FromPointAndDirector(l2.pBase + nrm * 0.1, d2);
    EXPECT_FALSE(intersect(l1, l3, obj));
  }
}

TEST(GeometryIntersect, Polygon3DTiltedVsSegmentAndRay)
{
  rng().randomize(1234);
  for (int k = 0; k < 100; k++)
  {
    const TPose3D pose = randomPose();
    TPolygon3D sq;
    for (const auto& p : std::vector<TPoint3D>{
             {-1, -1, 0},
             { 1, -1, 0},
             { 1,  1, 0},
             {-1,  1, 0}
    })
    {
      sq.push_back(pose.composePoint(p));
    }
    const TPoint3D local(rng().drawUniform(-0.9, 0.9), rng().drawUniform(-0.9, 0.9), 0);
    const TPoint3D target = pose.composePoint(local);

    // Segment crossing the polygon plane at "target":
    const TSegment3D s(
        pose.composePoint(local + TPoint3D(0.2, -0.1, -1)),
        pose.composePoint(local + TPoint3D(-0.2, 0.1, 1)));
    TObject3D obj;
    ASSERT_TRUE(intersect(sq, s, obj));
    ASSERT_TRUE(obj.isPoint());
    EXPECT_NEAR((obj.getAs<TPoint3D>() - target).norm(), 0.0, 1e-9);

    // Same segment, shifted outside of the polygon:
    const TSegment3D sOut(
        pose.composePoint(TPoint3D(3, 0, -1)), pose.composePoint(TPoint3D(3, 0, 1)));
    EXPECT_FALSE(intersect(sq, sOut, obj));

    // Ray from an arbitrary point towards "target":
    const TPoint3D origin(
        rng().drawUniform(-20, 20), rng().drawUniform(-20, 20), rng().drawUniform(-20, 20));
    const TPoint3D d = target - origin;
    const TPose3D ray(
        origin.x, origin.y, origin.z, std::atan2(d.y, d.x), -std::atan2(d.z, std::hypot(d.x, d.y)),
        0);
    double dist = 0;
    ASSERT_TRUE(traceRay(std::vector<TPolygon3D>{sq}, ray, dist));
    EXPECT_NEAR(dist, d.norm(), 1e-8);
  }
}

TEST(GeometryIntersect, Object3DPlaneVsPolygonIsSymmetric)
{
  // Both argument orders must give the same result.
  const TPolygon3D poly{
      {-1, -1, -1},
      { 1, -1,  1},
      { 1,  1,  1},
      {-1,  1, -1}
  };
  const TPlane pl(0, 0, 1, 0);
  TObject3D obj1;
  TObject3D obj2;
  ASSERT_TRUE(intersect(TObject3D::From(poly), TObject3D::From(pl), obj1));
  EXPECT_TRUE(intersect(TObject3D::From(pl), TObject3D::From(poly), obj2));
  EXPECT_EQ(obj1.asString(), obj2.asString());
}
