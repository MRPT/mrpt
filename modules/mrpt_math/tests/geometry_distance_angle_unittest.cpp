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
#include <mrpt/math/TPlane.h>
#include <mrpt/math/TPolygon3D.h>
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

// Point-line distance via the cross product (independent of TLine3D).
double lineDist(const TLine3D& l, const TPoint3D& p)
{
  return (p - l.pBase).cross(l.director).norm() / l.director.norm();
}

TPoint3D randomPoint(double r)
{
  return {rng().drawUniform(-r, r), rng().drawUniform(-r, r), rng().drawUniform(-r, r)};
}

double pointSegmentDistance(const TPoint3D& p, const TPoint3D& a, const TPoint3D& b)
{
  const TPoint3D ab = b - a;
  const double t = std::clamp((p - a).dot(ab) / ab.sqrNorm(), 0.0, 1.0);
  return (p - (a + ab * t)).norm();
}

// Independent segment-segment distance: interior stationary point of the
// convex quadratic if feasible, otherwise the best of the four boundaries.
double refSegmentSegmentDistance(const TSegment3D& s1, const TSegment3D& s2)
{
  const TPoint3D u = s1.point2 - s1.point1;
  const TPoint3D v = s2.point2 - s2.point1;
  const TPoint3D w = s1.point1 - s2.point1;
  const double a = u.dot(u);
  const double b = u.dot(v);
  const double c = v.dot(v);
  const double d = u.dot(w);
  const double e = v.dot(w);
  const double den = a * c - b * b;
  double best = std::min(
      {pointSegmentDistance(s1.point1, s2.point1, s2.point2),
       pointSegmentDistance(s1.point2, s2.point1, s2.point2),
       pointSegmentDistance(s2.point1, s1.point1, s1.point2),
       pointSegmentDistance(s2.point2, s1.point1, s1.point2)});
  if (den > 1e-12 * a * c)
  {
    const double s = (b * e - c * d) / den;
    const double t = (a * e - b * d) / den;
    if (s >= 0 && s <= 1 && t >= 0 && t <= 1)
    {
      best = std::min(best, (w + u * s - v * t).norm());
    }
  }
  return best;
}
}  // namespace

TEST(GeometryDistance, AntiparallelLines3D)
{
  // Antiparallel directors must be handled as parallel lines.
  const TLine3D a = TLine3D::FromPointAndDirector({0, 0, 0}, {1, 0, 0});
  const TLine3D b = TLine3D::FromPointAndDirector({0, 1, 0}, {-1, 0, 0});
  EXPECT_NEAR(distance(a, b), 1.0, 1e-12);
  const TLine3D c = TLine3D::FromPointAndDirector({0, 1, 0}, {1, 0, 0});
  EXPECT_NEAR(distance(a, c), 1.0, 1e-12);
}

TEST(GeometryDistance, AntiparallelPlanes)
{
  // Antiparallel normals must be handled as parallel planes.
  const TPlane p1(0, 0, 1, 0);
  const TPlane p2(0, 0, -1, 1);  // z = 1, normal pointing down
  EXPECT_NEAR(distance(p1, p2), 1.0, 1e-12);
  const TPlane p3(0, 0, 2, -2);  // z = 1, normal pointing up
  EXPECT_NEAR(distance(p1, p3), 1.0, 1e-12);
}

TEST(GeometryDistance, Lines3DRandomVsTripleProduct)
{
  rng().randomize(1234);
  for (int k = 0; k < 1000; k++)
  {
    const TVector3D d1 = randomUnitVector() * rng().drawUniform(0.1, 10);
    const TVector3D d2 = randomUnitVector() * rng().drawUniform(0.1, 10);
    const TVector3D n = d1.cross(d2);
    if (n.norm() < 0.05 * d1.norm() * d2.norm())
    {
      continue;
    }
    const TLine3D l1 = TLine3D::FromPointAndDirector(randomPoint(5), d1);
    const TLine3D l2 = TLine3D::FromPointAndDirector(randomPoint(5), d2);
    const double expected = std::abs((l2.pBase - l1.pBase).dot(n)) / n.norm();
    EXPECT_NEAR(distance(l1, l2), expected, 1e-9);
    const auto d = l1.distance(l2);
    ASSERT_TRUE(d.has_value());
    EXPECT_NEAR(*d, expected, 1e-9);
  }
}

TEST(GeometryDistance, Segment3DRandomVsReference)
{
  rng().randomize(1234);
  for (int k = 0; k < 3000; k++)
  {
    const TSegment3D s1(randomPoint(3), randomPoint(3));
    const TSegment3D s2(randomPoint(3), randomPoint(3));
    if (s1.length() < 0.1 || s2.length() < 0.1)
    {
      continue;
    }
    EXPECT_NEAR(s1.distance(s2), refSegmentSegmentDistance(s1, s2), 1e-9) << s1 << " " << s2;
  }
}

TEST(GeometryDistance, Segment3DShortSegments)
{
  // The result must not depend on the segments' scale.
  const TSegment3D a({-4e-3, 0, 0}, {4e-3, 0, 0});
  const TSegment3D b({0, -4e-3, 0}, {0, 4e-3, 0});
  EXPECT_NEAR(a.distance(b), 0.0, 1e-12);
  const TSegment3D c({-4e-3, 0, 1e-3}, {4e-3, 0, 1e-3});
  EXPECT_NEAR(c.distance(b), 1e-3, 1e-12);
}

TEST(GeometryDistance, SegmentContainsUsesADistanceTolerance)
{
  // The tolerance is a distance, independent of the segment length.
  const TSegment2D s2({0, 0}, {100, 0});
  EXPECT_FALSE(s2.contains({50, 0.02}));
  const TSegment3D s3({0, 0, 0}, {100, 0, 0});
  EXPECT_FALSE(s3.contains({50, 0, 0.02}));
  // Sanity: points on the segment are contained.
  EXPECT_TRUE(s2.contains({50, 0}));
  EXPECT_TRUE(s3.contains({50, 0, 0}));
}

TEST(GeometryDistance, Polygon3DInTiltedPlane)
{
  // Polygon in a plane whose pose is not a rotation about Z.
  const TPolygon3D poly{
      {0, 0, 0},
      {0, 1, 0},
      {0, 1, 1},
      {0, 0, 1}
  };
  EXPECT_NEAR(poly.distance({5, 0.5, 0.5}), 5.0, 1e-9);
  EXPECT_NEAR(poly.distance({0, 3, 0.5}), 2.0, 1e-9);
  EXPECT_NEAR(poly.distance({3, 0.5, 5}), 5.0, 1e-9);
}

TEST(GeometryAngle, PlaneAndLineAlongItsNormal)
{
  // Rounding may give |cos| slightly larger than 1 here.
  const TPlane pl(2.0, 7.4, 0.71, 0);
  const TLine3D ln = TLine3D::FromPointAndDirector({0, 0, 5}, {6.0, 22.2, 2.13});
  EXPECT_NEAR(std::abs(getAngle(pl, ln)), M_PI / 2, 1e-9);
  // Consequences in TPlane: the line crosses the plane, so it is not contained
  // and its distance is zero.
  EXPECT_FALSE(pl.contains(TLine3D::FromPointAndDirector({0, 0, 0}, {6.0, 22.2, 2.13})));
  EXPECT_NEAR(pl.distance(ln), 0.0, 1e-12);
}

TEST(GeometryAngle, PlaneLineRandomVsAsin)
{
  rng().randomize(1234);
  for (int k = 0; k < 1000; k++)
  {
    const TVector3D n = randomUnitVector() * rng().drawUniform(0.1, 10);
    const TVector3D d = randomUnitVector() * rng().drawUniform(0.1, 10);
    const TPlane pl(n.x, n.y, n.z, rng().drawUniform(-3, 3));
    const TLine3D ln = TLine3D::FromPointAndDirector(randomPoint(3), d);
    const double s = n.dot(d) / (n.norm() * d.norm());
    if (std::abs(s) > 0.999)
    {
      continue;
    }
    EXPECT_NEAR(getAngle(pl, ln), std::asin(s), 1e-9);
  }
}

TEST(GeometryAngle, Lines3DRandomInZeroPi)
{
  rng().randomize(1234);
  for (int k = 0; k < 1000; k++)
  {
    const TVector3D d1 = randomUnitVector() * rng().drawUniform(0.1, 10);
    const TVector3D d2 = randomUnitVector() * rng().drawUniform(0.1, 10);
    const TLine3D l1 = TLine3D::FromPointAndDirector(randomPoint(3), d1);
    const TLine3D l2 = TLine3D::FromPointAndDirector(randomPoint(3), d2);
    const double expected = std::atan2(d1.cross(d2).norm(), d1.dot(d2));
    EXPECT_NEAR(getAngle(l1, l2), expected, 1e-7);
  }
}

TEST(GeometryAngle, Lines2DRandomRelativeAngle)
{
  rng().randomize(1234);
  for (int k = 0; k < 1000; k++)
  {
    const double a1 = rng().drawUniform(-M_PI, M_PI);
    const double a2 = rng().drawUniform(-M_PI, M_PI);
    const TPoint2D p1(rng().drawUniform(-3, 3), rng().drawUniform(-3, 3));
    const TPoint2D p2(rng().drawUniform(-3, 3), rng().drawUniform(-3, 3));
    const double s1 = rng().drawUniform(0.1, 5);
    const double s2 = rng().drawUniform(0.1, 5);
    const TLine2D l1(p1, p1 + TPoint2D(std::cos(a1), std::sin(a1)) * s1);
    const TLine2D l2(p2, p2 + TPoint2D(std::cos(a2), std::sin(a2)) * s2);
    EXPECT_NEAR(getAngle(l1, l2), wrapToPi(a2 - a1), 1e-9);
  }
}

TEST(GeometryDistance, Line3DClosestPointRandom)
{
  rng().randomize(1234);
  for (int k = 0; k < 1000; k++)
  {
    const TVector3D d = randomUnitVector() * rng().drawUniform(0.1, 10);
    const TLine3D l = TLine3D::FromPointAndDirector(randomPoint(5), d);
    const TPoint3D p = randomPoint(10);
    const TPoint3D c = l.closestPointTo(p);
    // On the line, and the residual is orthogonal to the director:
    EXPECT_NEAR(lineDist(l, c), 0.0, 1e-9);
    EXPECT_NEAR((p - c).dot(d) / d.norm(), 0.0, 1e-9);
    EXPECT_NEAR((p - c).norm(), lineDist(l, p), 1e-9);
  }
}

TEST(GeometryDistance, Line3DDistanceToPointsOnTheLine)
{
  // Points on or very close to the line must not suffer from cancellation.
  rng().randomize(1234);
  double worst = 0;
  for (int k = 0; k < 1000; k++)
  {
    const TVector3D d = randomUnitVector() * rng().drawUniform(0.1, 10);
    const TLine3D l = TLine3D::FromPointAndDirector(randomPoint(5), d);
    const TPoint3D onLine = l.pBase + d * rng().drawUniform(-3, 3);
    const TVector3D nrm = d.cross(randomUnitVector()).unitarize();
    const double h = rng().drawUniform(0, 1e-6);
    const TPoint3D p = onLine + nrm * h;
    worst = std::max(worst, std::abs(l.distance(p) - lineDist(l, p)));
  }
  EXPECT_LT(worst, 1e-12);
}
