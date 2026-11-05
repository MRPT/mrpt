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
#include <mrpt/math/TLine3D.h>
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

// Independent even-odd ray casting (valid for simple polygons).
bool refContains(const TPolygon2D& poly, const TPoint2D& p)
{
  bool inside = false;
  const size_t n = poly.size();
  for (size_t i = 0, j = n - 1; i < n; j = i++)
  {
    const TPoint2D& a = poly[i];
    const TPoint2D& b = poly[j];
    if ((a.y > p.y) != (b.y > p.y))
    {
      const double xCross = a.x + (p.y - a.y) * (b.x - a.x) / (b.y - a.y);
      if (p.x < xCross)
      {
        inside = !inside;
      }
    }
  }
  return inside;
}

double distanceToBoundary(const TPolygon2D& poly, const TPoint2D& p)
{
  double d = 1e300;
  for (size_t i = 0; i < poly.size(); i++)
  {
    const TPoint2D q = closestFromPointToSegment(p, poly[i], poly[(i + 1) % poly.size()]);
    d = std::min(d, (q - p).norm());
  }
  return d;
}

TPolygon2D randomStarPolygon(size_t n, double rMin, double rMax)
{
  std::vector<double> ang;
  for (size_t i = 0; i < n; i++)
  {
    ang.push_back(rng().drawUniform(-M_PI, M_PI));
  }
  std::sort(ang.begin(), ang.end());
  TPolygon2D p;
  for (const double a : ang)
  {
    const double r = rng().drawUniform(rMin, rMax);
    p.emplace_back(r * std::cos(a), r * std::sin(a));
  }
  return p;
}

TPolygon2D reversed(TPolygon2D p)
{
  std::reverse(p.begin(), p.end());
  return p;
}
}  // namespace

TEST(GeometryPolygons, RemoveRepeatedVerticesSeveral)
{
  // More than one vertex removed.
  TPolygon2D p{
      {0, 0},
      {0, 0},
      {1, 0},
      {1, 0},
      {1, 1}
  };
  p.removeRepeatedVertices();
  const TPolygon2D expected{
      {0, 0},
      {1, 0},
      {1, 1}
  };
  ASSERT_EQ(p.size(), expected.size());
  for (size_t i = 0; i < p.size(); i++)
  {
    EXPECT_EQ(p[i], expected[i]) << "i=" << i;
  }
}

TEST(GeometryPolygons, RemoveRedundantVerticesSeveral)
{
  // More than one collinear vertex removed.
  TPolygon3D p{
      {0, 0, 0},
      {1, 0, 0},
      {2, 0, 0},
      {2, 1, 0},
      {2, 2, 0},
      {0, 2, 0}
  };
  p.removeRedundantVertices();
  const TPolygon3D expected{
      {0, 0, 0},
      {2, 0, 0},
      {2, 2, 0},
      {0, 2, 0}
  };
  ASSERT_EQ(p.size(), expected.size());
  for (size_t i = 0; i < p.size(); i++)
  {
    EXPECT_EQ(p[i], expected[i]) << "i=" << i;
  }
}

TEST(GeometryPolygons, AssemblePolygonsSkipsDegenerateSegment)
{
  // A zero-length segment must be ignored without shifting the others.
  const std::vector<TSegment3D> segs = {
      {{9, 9, 9}, {9, 9, 9}},
      {{0, 0, 0}, {1, 0, 0}},
      {{1, 0, 0}, {1, 1, 0}},
      {{1, 1, 0}, {0, 0, 0}}
  };
  std::vector<TPolygon3D> polys;
  assemblePolygons(segs, polys);
  ASSERT_EQ(polys.size(), 1u);
  ASSERT_EQ(polys[0].size(), 3u);
  for (const auto& v : polys[0])
  {
    EXPECT_LT(v.norm(), 2.0) << v;
  }
}

TEST(GeometryPolygons, CollinearPointsDoNotConformAPlane)
{
  // Collinear points: boolean queries must return false, not throw.
  const std::vector<TPoint3D> pts{
      {0, 0, 0},
      {1, 0, 0},
      {2, 0, 0}
  };
  TPlane pl;
  bool ok = true;
  EXPECT_NO_THROW(ok = conformAPlane(pts, pl));
  EXPECT_FALSE(ok);
  const TPolygon3D poly(pts);
  EXPECT_NO_THROW(ok = poly.getPlane(pl));
  EXPECT_FALSE(ok);
  TObject3D obj;
  EXPECT_NO_THROW(intersect(poly, TPlane(1, 0, 0, -0.5), obj));
}

TEST(GeometryPolygons, ContainsAgreesWithRayCasting)
{
  rng().randomize(1234);
  int nIn = 0;
  int nOut = 0;
  for (int k = 0; k < 100; k++)
  {
    const auto poly = randomStarPolygon(12, 0.3, 3.0);
    const auto polyRev = reversed(poly);
    for (int i = 0; i < 100; i++)
    {
      const TPoint2D p(rng().drawUniform(-3.5, 3.5), rng().drawUniform(-3.5, 3.5));
      if (distanceToBoundary(poly, p) < 1e-6)
      {
        continue;
      }
      const bool expected = refContains(poly, p);
      ASSERT_EQ(poly.contains(p), expected) << p;
      ASSERT_EQ(polyRev.contains(p), expected) << p;
      if (expected)
      {
        nIn++;
      }
      else
      {
        nOut++;
      }
    }
  }
  EXPECT_GT(nIn, 1000);
  EXPECT_GT(nOut, 1000);
}

TEST(GeometryPolygons, PointIntoQuadrangleConvex)
{
  rng().randomize(1234);
  for (int k = 0; k < 200; k++)
  {
    // Convex quadrangle: 4 sorted angles on an ellipse.
    const auto q = randomStarPolygon(4, 1.0, 1.0);
    TPolygon2D quad;
    for (const auto& v : q)
    {
      quad.emplace_back(2.0 * v.x + 0.5, 0.7 * v.y - 0.2);
    }
    for (int i = 0; i < 50; i++)
    {
      const TPoint2D p(rng().drawUniform(-3, 3), rng().drawUniform(-1.5, 1.5));
      if (distanceToBoundary(quad, p) < 1e-6)
      {
        continue;
      }
      EXPECT_EQ(
          pointIntoQuadrangle(
              p.x, p.y, quad[0].x, quad[0].y, quad[1].x, quad[1].y, quad[2].x, quad[2].y, quad[3].x,
              quad[3].y),
          refContains(quad, p));
    }
  }
}

TEST(GeometryPolygons, SplitInConvexComponentsPreservesArea)
{
  const std::vector<TPolygon2D> shapes = {
      {{0, 0},{2, 0},{2, 1},{1, 1},{1, 2},{0, 2}                                       },
      { {0, 0},{3, 0},{3, 3},{2, 3}, {2, 1},  {1, 1},{1, 3}, {0, 3}},
      { {0, 0}, {5, 0}, {5, 2}, {4, 1}, {3, 2},  {2, 1}, {1, 2}, {0, 1}},
      { {0, 0}, {2, 1}, {0, 2}, {1, 1}       },
      { {0, 3},
       {1, 1},
       {3, 1},
       {1.5, -0.5},
       {2, -3},
       {0, -1.5},                                 {-2, -3},
       {-1.5, -0.5},
       {-3, 1},
       {-1, 1}}
  };
  for (const auto& s0 : shapes)
  {
    for (const auto& s : {s0, reversed(s0)})
    {
      std::vector<TPolygon2D> comps;
      ASSERT_TRUE(splitInConvexComponents(s, comps));
      double area = 0;
      for (const auto& c : comps)
      {
        EXPECT_TRUE(c.isConvex());
        area += std::abs(signedArea(c));
        TPoint2D ctr;
        c.getCenter(ctr);
        EXPECT_TRUE(s.contains(ctr));
      }
      EXPECT_NEAR(area, std::abs(signedArea(s)), 1e-9);
    }
  }
}

TEST(GeometryPolygons, SignedAreaOfRegularPolygons)
{
  for (size_t n = 3; n < 12; n++)
  {
    const double r = 1.7;
    TPolygon2D p;
    TPolygon2D::createRegularPolygon(n, r, p, TPose2D(2.0, -1.0, 0.3));
    const double expected =
        0.5 * static_cast<double>(n) * r * r * std::sin(2 * M_PI / static_cast<double>(n));
    EXPECT_NEAR(signedArea(p), expected, 1e-12);
    EXPECT_NEAR(signedArea(reversed(p)), -expected, 1e-12);
    EXPECT_TRUE(p.isConvex());
  }
}

TEST(GeometryPolygons, Polygon3DContainsInTiltedPlane)
{
  rng().randomize(1234);
  for (int k = 0; k < 100; k++)
  {
    const TPose3D pose(
        rng().drawUniform(-5, 5), rng().drawUniform(-5, 5), rng().drawUniform(-5, 5),
        rng().drawUniform(-3, 3), rng().drawUniform(-1.5, 1.5), rng().drawUniform(-3, 3));
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
    const double x = rng().drawUniform(-0.95, 0.95);
    const double y = rng().drawUniform(-0.95, 0.95);
    EXPECT_TRUE(sq.contains(pose.composePoint(TPoint3D(x, y, 0))));
    EXPECT_FALSE(sq.contains(pose.composePoint(TPoint3D(x, y, 0.01))));
    EXPECT_FALSE(sq.contains(pose.composePoint(TPoint3D(x + 2.0, y, 0))));
  }
}

TEST(GeometryPolygons, AreAligned3DWithCloseRepeatedPoints)
{
  // All points within the TLine3D epsilon of the first one.
  const std::vector<TPoint3D> pts{
      {   0, 0, 0},
      {1e-7, 0, 0}
  };
  TLine3D r;
  if (areAligned(pts, r))
  {
    // Whatever line is returned, it must pass through the first point and run
    // along the X axis.
    const double n = r.director.norm();
    ASSERT_TRUE(std::isfinite(n) && n > 0) << r.asString();
    EXPECT_NEAR(std::abs(r.director.x) / n, 1.0, 1e-9) << r.asString();
    EXPECT_NEAR(r.pBase.norm(), 0.0, 1e-6) << r.asString();
  }
}
