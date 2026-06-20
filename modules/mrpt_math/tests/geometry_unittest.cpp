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
#include <mrpt/containers/yaml.h>
#include <mrpt/math/CPolygon.h>
#include <mrpt/math/TLine3D.h>
#include <mrpt/math/TObject2D.h>
#include <mrpt/math/TObject3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/geometry.h>

#include <algorithm>

using namespace mrpt;
using namespace mrpt::math;
using namespace std;

TEST(Geometry, Line2DIntersect)
{
  // Two lines that should intersect at (0.5,0.5)
  const TLine2D l1(TPoint2D(0, 1), TPoint2D(1, 0));
  const TLine2D l2(TPoint2D(-1, 0.5), TPoint2D(4, 0.5));

  TObject2D inter;
  bool do_inter = intersect(l1, l2, inter);

  EXPECT_TRUE(do_inter);
  EXPECT_TRUE(inter.isPoint());

  TPoint2D i(0, 0);
  inter.getPoint(i);
  EXPECT_NEAR(i.x, 0.5, 1e-9);
  EXPECT_NEAR(i.y, 0.5, 1e-9);
}

TEST(Geometry, Line2DAngle)
{
  const TLine2D l1(TPoint2D(0, 0), TPoint2D(1, 0));
  const TLine2D l2(TPoint2D(-1, -1), TPoint2D(5, 5));

  // Angles in 2D do have sign:
  EXPECT_NEAR(mrpt::RAD2DEG(mrpt::math::getAngle(l1, l2)), +45.0, 1e-5);
  EXPECT_NEAR(mrpt::RAD2DEG(mrpt::math::getAngle(l2, l1)), -45.0, 1e-5);

  EXPECT_NEAR(mrpt::RAD2DEG(mrpt::math::getAngle(l1, l1)), 0.0, 1e-5);
  EXPECT_NEAR(mrpt::RAD2DEG(mrpt::math::getAngle(l2, l2)), 0.0, 1e-5);

  const TLine2D l3(TPoint2D(1, 0), TPoint2D(0, 0));
  EXPECT_NEAR(RAD2DEG(std::abs(mrpt::math::getAngle(l1, l3))), 180.0, 1e-5);
  EXPECT_NEAR(RAD2DEG(std::abs(mrpt::math::getAngle(l3, l1))), 180.0, 1e-5);
}

TEST(Geometry, Line3DAngle)
{
  const TLine3D l1(TPoint3D(0, 0, 0), TPoint3D(1, 0, 0));
  const TLine3D l2(TPoint3D(-1, -1, 0), TPoint3D(5, 5, 0));

  // Angles in 3D don't have sign:
  EXPECT_NEAR(mrpt::RAD2DEG(mrpt::math::getAngle(l1, l2)), 45.0, 1e-5);
  EXPECT_NEAR(mrpt::RAD2DEG(mrpt::math::getAngle(l2, l1)), 45.0, 1e-5);

  EXPECT_NEAR(mrpt::RAD2DEG(mrpt::math::getAngle(l1, l1)), 0.0, 1e-5);
  EXPECT_NEAR(mrpt::RAD2DEG(mrpt::math::getAngle(l2, l2)), 0.0, 1e-5);

  const TLine3D l3(TPoint3D(1, 0, 0), TPoint3D(0, 0, 0));
  EXPECT_NEAR(RAD2DEG(std::abs(mrpt::math::getAngle(l1, l3))), 180.0, 1e-5);
  EXPECT_NEAR(RAD2DEG(std::abs(mrpt::math::getAngle(l3, l1))), 180.0, 1e-5);

  const TLine3D l4(TPoint3D(0, 0, 0), TPoint3D(cos(30.0_deg), sin(30.0_deg), 0));
  EXPECT_NEAR(mrpt::RAD2DEG(mrpt::math::getAngle(l1, l4)), 30.0, 1e-5);
  EXPECT_NEAR(mrpt::RAD2DEG(mrpt::math::getAngle(l4, l1)), 30.0, 1e-5);
}

TEST(Geometry, PlaneAngle)
{
  const auto p1 = TPlane::From3Points({0, 0, 0}, {1, 0, 0}, {0, 0, 1});
  const auto p2 = TPlane::From3Points({0, 0, 0}, {1, 1, 1}, {0, 1, 0});
  EXPECT_NEAR(mrpt::math::getAngle(p1, p2), 90.0_deg, 1e-4);

  const auto p3 = TPlane::From3Points({0, 0, 10}, {1, 1, 11}, {0, 1, 10});
  EXPECT_NEAR(mrpt::math::getAngle(p1, p3), 90.0_deg, 1e-4);
  EXPECT_NEAR(mrpt::math::getAngle(p2, p3), 0.0_deg, 1e-4);

  EXPECT_NEAR(
      mrpt::math::getAngle(
          TPlane::FromPointAndNormal({0.0, 0.0, 0.0}, {0.0, +1.0, 0.0}),
          TPlane::FromPointAndNormal({4.0, 4.0, 4.0}, {0.0, -1.0, 0.0})),
      180.0_deg, 1e-4);
  EXPECT_NEAR(
      mrpt::math::getAngle(
          TPlane::FromPointAndNormal({0.0, 0.0, 0.0}, {0.0, -1.0, 0.0}),
          TPlane::FromPointAndNormal({4.0, 4.0, 4.0}, {0.0, -1.0, 0.0})),
      0.0_deg, 1e-4);
}

TEST(Geometry, PlaneLineAngle)
{
  EXPECT_NEAR(
      mrpt::math::getAngle(
          TPlane::FromPointAndNormal({0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}),
          TLine3D::FromTwoPoints({0.0, 0.0, 4.0}, {0.0, 0.0, 5.0})),
      90.0_deg, 1e-4);
  EXPECT_NEAR(
      mrpt::math::getAngle(
          TPlane::FromPointAndNormal({0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}),
          TLine3D::FromTwoPoints({0.0, 0.0, 4.0}, {1.0, 0.0, 4.0})),
      0.0_deg, 1e-4);
  EXPECT_NEAR(
      mrpt::math::getAngle(
          TPlane::FromPointAndNormal({0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}),
          TLine3D::FromTwoPoints({0.0, 0.0, 4.0}, {1.0, 0.0, 5.0})),
      45.0_deg, 1e-4);
}

TEST(Geometry, PlanePointDistance)
{
  EXPECT_NEAR(
      TPlane::FromPointAndNormal({0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}).distance({0, 0, 0}), 0.0, 1e-4);
  EXPECT_NEAR(
      TPlane::FromPointAndNormal({10.0, 20.0, 30.0}, {0.0, 0.0, 1.0}).distance({11.0, 20.0, 30.0}),
      0.0, 1e-4);
  EXPECT_NEAR(
      TPlane::FromPointAndNormal({10.0, 20.0, 30.0}, {0.0, 0.0, -1.0}).distance({11.0, 21.0, 31.0}),
      1.0, 1e-4);
  EXPECT_NEAR(
      TPlane::FromPointAndNormal({10.0, 20.0, 30.0}, {0.0, 0.0, +1.0}).distance({11.0, 21.0, 31.0}),
      1.0, 1e-4);
  EXPECT_NEAR(
      TPlane::FromPointAndNormal({10.0, 20.0, 30.0}, {0.0, 0.0, +5.0}).distance({11.0, 21.0, 31.0}),
      1.0, 1e-4);
}
TEST(Geometry, PlanePointSignedDistance)
{
  EXPECT_NEAR(
      TPlane::FromPointAndNormal({0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}).signedDistance({0, 0, 0}), 0.0,
      1e-4);
  EXPECT_NEAR(
      TPlane::FromPointAndNormal({10.0, 20.0, 30.0}, {0.0, 0.0, 1.0})
          .signedDistance({11.0, 20.0, 30.0}),
      0.0, 1e-4);
  EXPECT_NEAR(
      TPlane::FromPointAndNormal({10.0, 20.0, 30.0}, {0.0, 0.0, -1.0})
          .signedDistance({11.0, 21.0, 31.0}),
      -1.0, 1e-4);
  EXPECT_NEAR(
      TPlane::FromPointAndNormal({10.0, 20.0, 30.0}, {0.0, 0.0, +5.0})
          .signedDistance({11.0, 21.0, 31.0}),
      +1.0, 1e-4);
}

TEST(Geometry, Line3DDistance)
{
  {
    // intersect
    const auto l1 = TLine3D::FromTwoPoints({0, 0, 0}, {1, 0, 0});
    const auto l2 = TLine3D::FromTwoPoints({-1, -1, 0}, {5, 5, 0});
    EXPECT_NEAR(l1.distance(l2).value(), .0, 1e-10);
  }
  {
    // cross (do not intersect)
    const auto l1 = TLine3D::FromTwoPoints({0, 0, 0}, {1, 0, 0});
    const auto l2 = TLine3D::FromTwoPoints({-1, -1, 1}, {5, 5, 1});
    EXPECT_NEAR(l1.distance(l2).value(), 1.0, 1e-10);
  }
  {
    // parallel:
    const auto l1 = TLine3D::FromTwoPoints({0, 0, 0}, {1, 0, 0});
    const auto l2 = TLine3D::FromTwoPoints({0, 2, 0}, {2, 2, 0});
    EXPECT_FALSE(l1.distance(l2).has_value());
  }
}

TEST(Geometry, Line3DDclosestPointTo)
{
  {
    const auto l = TLine3D::FromTwoPoints({0, 0, 0}, {1, 0, 0});
    const auto p = TPoint3D(-1, -1, 1);
    const auto pc = l.closestPointTo(p);

    EXPECT_NEAR(pc.x, -1, 1e-8);
    EXPECT_NEAR(pc.y, 0, 1e-8);
    EXPECT_NEAR(pc.z, 0, 1e-8);
  }
  {
    const auto l = TLine3D::FromTwoPoints({0, 0, 0}, {0, 0, 1});
    const auto p = TPoint3D(0, 0, -2);
    const auto pc = l.closestPointTo(p);

    EXPECT_NEAR(pc.x, 0, 1e-8);
    EXPECT_NEAR(pc.y, 0, 1e-8);
    EXPECT_NEAR(pc.z, -2, 1e-8);
  }
}

TEST(Geometry, Segment2DIntersect)
{
  {
    // Two segments that should intersect at (0.5,0.5)
    const TSegment2D s1(TPoint2D(0, 1), TPoint2D(1, 0));
    const TSegment2D s2(TPoint2D(-1, 0.5), TPoint2D(4, 0.5));

    TObject2D inter;
    bool do_inter = intersect(s1, s2, inter);

    EXPECT_TRUE(do_inter);
    EXPECT_TRUE(inter.isPoint());

    TPoint2D i(0, 0);
    inter.getPoint(i);
    EXPECT_NEAR(i.x, 0.5, 1e-9);
    EXPECT_NEAR(i.y, 0.5, 1e-9);
  }

  {
    // Two segments that do NOT intersect
    const TSegment2D s1(TPoint2D(0, 1), TPoint2D(1, 0));
    const TSegment2D s2(TPoint2D(0.6, 0.5), TPoint2D(4, 0.5));

    TObject2D inter;
    bool do_inter = intersect(s1, s2, inter);

    EXPECT_FALSE(do_inter);
  }
  {
    // Two parallel segments that do NOT intersect: result is a "segment in
    // the middle".
    const TSegment2D s1(TPoint2D(-0.05, 0.05), TPoint2D(-0.05, -0.05));
    const TSegment2D s2(TPoint2D(0, 0.135), TPoint2D(0, -0.0149999));

    TObject2D inter;
    bool do_inter = intersect(s1, s2, inter);

    // EXPECT_TRUE(do_inter && inter.getType()==GeometricEntity::SEGMENT);
    EXPECT_FALSE(do_inter);
  }
}

TEST(Geometry, Intersection3D)
{
  {
    TPolygon3D p3d({
        {1, 0, 0},
        {0, 1, 0},
        {0, 0, 1}
    });
    TSegment3D s3d({
        {1, 0, 0},
        {0, 1, 0},
    });

    TObject3D inter;
    EXPECT_TRUE(intersect(p3d, s3d, inter));
    EXPECT_TRUE(inter.isSegment());
    TSegment3D test;
    inter.getSegment(test);
    // Should this be true? EXPECT_EQ(s3d, test);
  }
  {
    TPolygon3D p3d({
        {1, 0, 0},
        {0, 1, 0},
        {0, 0, 1}
    });
    TSegment3D s3d({
        {0, 0, 0},
        {1, 1, 1},
    });

    TObject3D inter;
    EXPECT_TRUE(intersect(p3d, s3d, inter));
    EXPECT_TRUE(inter.isPoint());
  }
  {
    TSegment3D s3d1({
        {1, 0, 0},
        {0, 1, 0},
    });
    TSegment3D s3d2({
        {2, -1.0, 0},
        {0,  1.0, 0},
    });

    TObject3D inter;
    EXPECT_TRUE(intersect(s3d1, s3d2, inter));
    EXPECT_TRUE(inter.isSegment());
  }
  {
    TSegment3D s3d1({
        {1, 0, 0},
        {0, 1, 0},
    });
    TSegment3D s3d2({
        {0, 0, 0},
        {1, 1, 0},
    });

    TObject3D inter;
    EXPECT_TRUE(intersect(s3d1, s3d2, inter));
    EXPECT_TRUE(inter.isPoint());

    TPoint3D test;
    TPoint3D expect{0.5, 0.5, 0};
    inter.getPoint(test);
    EXPECT_EQ(expect, test);
  }
}

TEST(Geometry, IntersectionPlanePlane)
{
  {
    // Parallel planes
    TPlane plane1({
        {1, 0, 0},
        {0, 1, 0},
        {0, 0, 1},
    });
    TPlane plane2({
        {2, 0, 0},
        {0, 2, 0},
        {0, 0, 2},
    });

    TObject3D inter;
    EXPECT_FALSE(intersect(plane1, plane2, inter));
  }
  {
    // Same plane
    TPlane plane1({
        {1, 0, 0},
        {0, 1, 0},
        {0, 0, 1},
    });
    TPlane plane2({
        {-1,  1,  1},
        { 1, -1,  1},
        { 1,  1, -1},
    });

    TObject3D inter;
    EXPECT_TRUE(intersect(plane1, plane2, inter));
    EXPECT_TRUE(inter.isPlane());
  }
  {
    // Intersecting planes
    TPlane plane1({
        {1, 0, 0},
        {0, 1, 0},
        {0, 0, 1},
    });
    TPlane plane2({
        {1,  0,  0},
        {0, -1,  0},
        {0,  0, -1},
    });

    TObject3D inter;
    EXPECT_TRUE(intersect(plane1, plane2, inter));
    EXPECT_TRUE(inter.isLine());
  }
}

void myTestPolygonContainsPoint(std::vector<TPoint2D>& vs, bool convex)
{
  const mrpt::math::TPolygon2D poly(vs);

  EXPECT_EQ(poly.isConvex(), convex);

  EXPECT_TRUE(poly.contains(TPoint2D(0.0, 0.0)));
  EXPECT_TRUE(poly.contains(TPoint2D(0.0, 0.9)));
  EXPECT_TRUE(poly.contains(TPoint2D(-0.9, -0.9)));
  EXPECT_TRUE(poly.contains(TPoint2D(0.9, -0.9)));

  EXPECT_FALSE(poly.contains(TPoint2D(-4.0, -5.1)));
  EXPECT_FALSE(poly.contains(TPoint2D(-5.0, -0.1)));
  EXPECT_FALSE(poly.contains(TPoint2D(1.1, -6.1)));
  EXPECT_FALSE(poly.contains(TPoint2D(0, 5.1)));
  EXPECT_FALSE(poly.contains(TPoint2D(0, -1.1)));
}

TEST(Geometry, PolygonConvexContainsPoint)
{
  // Test with a polygon in one winding order:
  std::vector<TPoint2D> vs;
  vs.emplace_back(-1.0, -1.0);
  vs.emplace_back(0.0, 1.0);
  vs.emplace_back(1.0, -1.0);
  myTestPolygonContainsPoint(vs, true);

  // and the other:
  std::reverse(vs.begin(), vs.end());
  myTestPolygonContainsPoint(vs, true);

  {
    mrpt::math::CPolygon p;
    p.add_vertex(0, -0.322);
    p.add_vertex(-0.644, -0.322);
    p.add_vertex(-0.210377, -0.324673);
    p.add_vertex(0.433623, -0.324673);

    EXPECT_FALSE(p.contains(TPoint2D(0.73175, -0.325796)));
  }
}

TEST(Geometry, PolygonConcaveContainsPoint)
{
  // Test with a polygon in one winding order:
  std::vector<TPoint2D> vs;
  vs.emplace_back(-2.0, 3.0);
  vs.emplace_back(2.0, 2.0);
  vs.emplace_back(3.0, -4.0);
  vs.emplace_back(0.1, -3.0);
  vs.emplace_back(0.1, -0.1);
  vs.emplace_back(-0.1, -0.1);
  vs.emplace_back(-0.1, -3.0);
  vs.emplace_back(-2.0, -2.0);

  myTestPolygonContainsPoint(vs, false);

  // and the other:
  std::reverse(vs.begin(), vs.end());
  myTestPolygonContainsPoint(vs, false);
}

TEST(Geometry, changeEpsilon)
{
  // Default value:
  const double default_val = 1e-5;
  EXPECT_NEAR(mrpt::math::getEpsilon(), default_val, 1e-9);

  // Test changing:
  mrpt::math::setEpsilon(0.1);
  EXPECT_NEAR(mrpt::math::getEpsilon(), 0.1, 1e-9);

  // Test actual effects of epsilon:
  {
    const auto l1 = TLine2D({0.0, 0.0}, {1.0, 0.0});
    const auto l2 = TLine2D({0.0, 2.0}, {1.0, 2.0001});

    TObject2D obj;
    mrpt::math::setEpsilon(0.1);
    EXPECT_FALSE(mrpt::math::intersect(l1, l2, obj));
    mrpt::math::setEpsilon(1e-10);
    EXPECT_TRUE(mrpt::math::intersect(l1, l2, obj));
  }

  // Reset
  mrpt::math::setEpsilon(default_val);
}

TEST(Geometry, conformAPlane)
{
  {
    std::vector<TPoint3D> pts = {
        {0., 0., 0.},
        {1., 0., 0.},
        {1., 1., 0.},
        {0., 1., 0.}
    };
    EXPECT_TRUE(mrpt::math::conformAPlane(pts));
  }
  {
    std::vector<TPoint3D> pts = {
        {0., 0., 0.},
        {0., 0., 1.},
        {0., 1., 1.},
        {0., 1., 0.}
    };
    EXPECT_TRUE(mrpt::math::conformAPlane(pts));
  }
  {
    std::vector<TPoint3D> pts = {
        { 0., 0.,  0.},
        { 0., 0.,  1.},
        { 0., 1.,  1.},
        {0.1, 1., 0.1}
    };
    EXPECT_FALSE(mrpt::math::conformAPlane(pts));
  }
  {
    std::vector<TPoint3D> pts = {
        { 5.56496063, -2.30508217, 29.53900000},
        { 5.87949871,  0.00000000, 29.53900000},
        {13.50000000,  0.00000000,  0.00000000},
        {12.50465807, -7.29433126,  0.00000000}
    };
    EXPECT_TRUE(mrpt::math::conformAPlane(pts));
  }
}

TEST(Geometry, RectanglesIntersection)
{
  const auto r1_xmin = -1.0, r1_xmax = 1.0, r1_ymin = -1.0, r1_ymax = 1.0;
  const auto r2_xmin = -2.0, r2_xmax = 2.0, r2_ymin = -3.0, r2_ymax = 3.0;

  using mrpt::math::RectanglesIntersection;

  using tst_set_t = std::array<double, 4>;

  // Test cases: x,y,phi,  0/1:false/true (expected output)
  const std::vector<tst_set_t> tsts = {
      {   0,    0,   0.0_deg, /*result*/ 1},
      { 3.1,    0,   0.0_deg, /*result*/ 0},
      {-3.1,    0,   0.0_deg, /*result*/ 0},
      { 2.9,    0,   0.0_deg, /*result*/ 1},
      {-2.9,    0,   0.0_deg, /*result*/ 1},
      {   0,  4.1,   0.0_deg, /*result*/ 0},
      {   0,  3.9,   0.0_deg, /*result*/ 1},
      {   0, -4.1,   0.0_deg, /*result*/ 0},
      {   0, -3.9,   0.0_deg, /*result*/ 1},
      { 3.1,    0,   0.0_deg, /*result*/ 0},
      { 3.1,    0,  45.0_deg, /*result*/ 1},
      { 3.1,    0, -90.0_deg, /*result*/ 1}
  };

  for (const auto& t : tsts)
  {
    const auto p = mrpt::math::TPose2D(t[0], t[1], t[2]);
    EXPECT_EQ(
        RectanglesIntersection(
            r1_xmin, r1_xmax, r1_ymin, r1_ymax, r2_xmin, r2_xmax, r2_ymin, r2_ymax, p.x, p.y,
            p.phi),
        t[3] != 0.0);
  }
}

TEST(TLine2D, asString)
{
  const auto l = TLine2D::FromTwoPoints({1, 1}, {2, 2});
  const auto s = l.asString();
  EXPECT_EQ(s, "[   1.00000,   -1.00000,    0.00000]");
}

TEST(TLine3D, asString)
{
  const auto l = TLine3D::FromTwoPoints({1, 1, 1}, {2, 0, 0});
  const auto s = l.asString();
  EXPECT_EQ(
      s,
      "P=[   1.00000,    1.00000,    1.00000] u=[   1.00000,   -1.00000,   "
      "-1.00000]");
}

TEST(TPlane, asString)
{
  const auto p = TPlane::From3Points({1, 1, 1}, {2, 0, 0}, {0, 0, 5});
  const auto s = p.asString();
  EXPECT_EQ(s, "[  -5.00000,   -3.00000,   -2.00000,   10.00000]");
}

TEST(Geometry, closestFromPointToSegment)
{
  using mrpt::math::closestFromPointToSegment;
  using mrpt::math::TPoint2D;

  EXPECT_NEAR(
      (closestFromPointToSegment({0, 0}, {0, 1}, {1, 1}) - TPoint2D(0, 1)).norm(), .0, 1e-5);
  EXPECT_NEAR(
      (closestFromPointToSegment({0, 1}, {0, 1}, {1, 1}) - TPoint2D(0, 1)).norm(), .0, 1e-5);
  EXPECT_NEAR(
      (closestFromPointToSegment({1, 1}, {0, 1}, {1, 1}) - TPoint2D(1, 1)).norm(), .0, 1e-5);
  EXPECT_NEAR(
      (closestFromPointToSegment({-1, 2}, {0, 1}, {1, 1}) - TPoint2D(0, 1)).norm(), .0, 1e-5);
  EXPECT_NEAR(
      (closestFromPointToSegment({3, 3}, {0, 1}, {1, 1}) - TPoint2D(1, 1)).norm(), .0, 1e-5);
  EXPECT_NEAR(
      (closestFromPointToSegment({0.5, 0.1}, {0, 1}, {1, 1}) - TPoint2D(0.5, 1)).norm(), .0, 1e-5);
}

TEST(Geometry, closestFromPointToLine)
{
  using mrpt::math::closestFromPointToLine;
  using mrpt::math::TPoint2D;

  EXPECT_NEAR((closestFromPointToLine({0, 0}, {0, 1}, {1, 1}) - TPoint2D(0, 1)).norm(), .0, 1e-5);
  EXPECT_NEAR((closestFromPointToLine({0, 1}, {0, 1}, {1, 1}) - TPoint2D(0, 1)).norm(), .0, 1e-5);
  EXPECT_NEAR((closestFromPointToLine({1, 1}, {0, 1}, {1, 1}) - TPoint2D(1, 1)).norm(), .0, 1e-5);
  EXPECT_NEAR((closestFromPointToLine({-1, 2}, {0, 1}, {1, 1}) - TPoint2D(-1, 1)).norm(), .0, 1e-5);
  EXPECT_NEAR((closestFromPointToLine({3, 3}, {0, 1}, {1, 1}) - TPoint2D(3, 1)).norm(), .0, 1e-5);
  EXPECT_NEAR(
      (closestFromPointToLine({0.5, 0.1}, {0, 1}, {1, 1}) - TPoint2D(0.5, 1)).norm(), .0, 1e-5);
}

TEST(Geometry, squaredDistancePointToLine)
{
  using mrpt::math::squaredDistancePointToLine;
  using mrpt::math::TPoint2D;
  EXPECT_NEAR(squaredDistancePointToLine({0, 0}, {0, 1}, {1, 1}), 1.0, 1e-5);
  EXPECT_NEAR(squaredDistancePointToLine({0, 1}, {0, 1}, {1, 1}), .0, 1e-5);
  EXPECT_NEAR(squaredDistancePointToLine({1, 1}, {0, 1}, {1, 1}), .0, 1e-5);
  EXPECT_NEAR(squaredDistancePointToLine({-1, 2}, {0, 1}, {1, 1}), 1.0, 1e-5);
  EXPECT_NEAR(squaredDistancePointToLine({3, 3}, {0, 1}, {1, 1}), 4.0, 1e-5);
  EXPECT_NEAR(squaredDistancePointToLine({0.5, 0.1}, {0, 1}, {1, 1}), 0.9 * 0.9, 1e-5);
}

TEST(TPolygon2D, toFromYAML)
{
  const mrpt::math::TPolygon2D p = {
      {-6.0, 0.5},
      { 8.0, 2.0},
      {10.0, 4.0},
      {-7.0, 3.0}
  };

  const auto py = p.asYAML();

  EXPECT_TRUE(py.isSequence());
  EXPECT_TRUE(py.asSequence().at(0).isSequence());

  const auto p2 = mrpt::math::TPolygon2D::FromYAML(py);
  EXPECT_EQ(p, p2);

  EXPECT_ANY_THROW(mrpt::math::TPolygon3D::FromYAML(py));

  EXPECT_TRUE(mrpt::math::TPolygon2D::FromYAML(mrpt::containers::yaml::Sequence()).empty());
}

TEST(TPolygon3D, toFromYAML)
{
  const mrpt::math::TPolygon3D p = {
      {-6.0, 0.5, 1.0},
      { 8.0, 2.0, 0.0},
      {10.0, 4.0, 3.0}
  };

  const auto py = p.asYAML();

  EXPECT_TRUE(py.isSequence());
  EXPECT_TRUE(py.asSequence().at(0).isSequence());

  const auto p2 = mrpt::math::TPolygon3D::FromYAML(py);
  EXPECT_EQ(p, p2);

  EXPECT_ANY_THROW(mrpt::math::TPolygon2D::FromYAML(py));
  EXPECT_TRUE(mrpt::math::TPolygon3D::FromYAML(mrpt::containers::yaml::Sequence()).empty());
}

TEST(Geometry, polygonIntersection)
{
  using mrpt::math::TPoint2D;

  // Define the polygons:
  const mrpt::math::TPolygon2D subject = {
      {{0.0, 0.0}, {5.0, 0.0}, {7.0, 3.0}, {3.0, 6.0}, {-4.0, 4.0}, {-1.0, -1.0}}
  };

  mrpt::math::TPolygon2D clipping = {
      {{-6.0, 0.5}, {8.0, 2.0}, {10.0, 4.0}, {-7.0, 3.0}}
  };

  // Compute intersection #1 (in one winding order)
  {
    mrpt::math::TObject2D clippedObj;
    bool doIntersect = mrpt::math::intersect(subject, clipping, clippedObj);
    EXPECT_TRUE(doIntersect);

    mrpt::math::TPolygon2D clippedPoly;
    bool isPoly = clippedObj.getPolygon(clippedPoly);
    EXPECT_TRUE(isPoly);

    const auto expectedPoly = mrpt::math::TPolygon2D({
        { 6.205128205128205, 1.807692307692308},
        {               7.0,               3.0},
        { 5.981818181818181, 3.763636363636364},
        {-3.522727272727272, 3.204545454545455},
        {-2.147651006711409, 0.912751677852349}
    });

    EXPECT_EQ(clippedPoly.size(), expectedPoly.size());
    for (size_t i = 0; i < expectedPoly.size(); i++)
      EXPECT_NEAR((clippedPoly.at(i) - expectedPoly.at(i)).norm(), 0, 1e-3);
  }

  // Compute intersection #2 (in the other winding order)
  {
    std::reverse(clipping.begin(), clipping.end());

    const mrpt::math::TPolygon2D clippedPoly = mrpt::math::intersect(subject, clipping);

    const auto expectedPoly = mrpt::math::TPolygon2D({
        { 6.205128205128205, 1.807692307692308},
        {               7.0,               3.0},
        { 5.981818181818181, 3.763636363636364},
        {-3.522727272727272, 3.204545454545455},
        {-2.147651006711409, 0.912751677852349}
    });

    EXPECT_EQ(clippedPoly.size(), expectedPoly.size());
    for (size_t i = 0; i < expectedPoly.size(); i++)
      EXPECT_NEAR((clippedPoly.at(i) - expectedPoly.at(i)).norm(), 0, 1e-3);
  }
}

TEST(Geometry, areAligned_TPoint2D)
{
  {
    const std::vector<mrpt::math::TPoint2D> pts = {
        {0.0, 0.0},
        {1.0, 1.0},
        {2.0, 2.0}
    };

    EXPECT_TRUE(mrpt::math::areAligned(pts));
  }

  {
    const std::vector<mrpt::math::TPoint2D> pts = {
        {0.0, 0.0},
        {1.0, 1.0},
        {2.0, 2.1}
    };

    EXPECT_FALSE(mrpt::math::areAligned(pts));
  }
}

TEST(Geometry, areAligned_TPoint3D)
{
  {
    const std::vector<mrpt::math::TPoint3D> pts = {
        {0.0, 0.0, 0.0},
        {1.0, 1.0, 0.0},
        {2.0, 2.0, 0.0}
    };

    EXPECT_TRUE(mrpt::math::areAligned(pts));
  }

  {
    const std::vector<mrpt::math::TPoint3D> pts = {
        {0.0, 0.0, 0.0},
        {1.0, 1.0, 1.0},
        {2.0, 2.0, 3.1}
    };

    EXPECT_FALSE(mrpt::math::areAligned(pts));
  }
}

TEST(Geometry, project2D)
{
  using mrpt::math::TPoint2D;

  const mrpt::math::TPose2D pose = {1.0, 1.0, 90.0_deg};

  {
    const auto o = mrpt::math::project2D(TPoint2D(1.0, 0.0), pose);
    EXPECT_NEAR((o - mrpt::math::TPoint2D(1.0, 2.0)).norm(), 0.0, 1e-4);
  }
  {
    const auto o =
        mrpt::math::project2D(mrpt::math::TLine2D::FromTwoPoints({0.0, 0.0}, {1.0, 0.0}), pose);
    EXPECT_TRUE(o.contains({1.0, 1.0}));
    EXPECT_TRUE(o.contains({1.0, 2.0}));
  }
  {
    const auto o =
        mrpt::math::project2D(mrpt::math::TSegment2D::FromPoints({0.0, 0.0}, {1.0, 0.0}), pose);
    EXPECT_NEAR((o.point1 - TPoint2D(1.0, 1.0)).norm(), 0.0, 1e-5);
    EXPECT_NEAR((o.point2 - TPoint2D(1.0, 2.0)).norm(), 0.0, 1e-5);
    EXPECT_TRUE(o.contains({1.0, 2.0}));
  }
  {
    const auto o = mrpt::math::project2D(
        mrpt::math::TPolygon2D({
            {0.0, 0.0},
            {1.0, 0.0},
            {0.0, 1.0}
    }),
        pose);
    EXPECT_NEAR((o.at(0) - TPoint2D(1.0, 1.0)).norm(), 0.0, 1e-5);
    EXPECT_NEAR((o.at(1) - TPoint2D(1.0, 2.0)).norm(), 0.0, 1e-5);
    EXPECT_NEAR((o.at(2) - TPoint2D(0.0, 1.0)).norm(), 0.0, 1e-5);
  }
}
TEST(Geometry, project3D)
{
  using mrpt::math::TPoint3D;
  const mrpt::math::TPose3D pose = {1.0, 1.0, 1.0, 90.0_deg, 0.0_deg, 0.0_deg};

  {
    const auto o = mrpt::math::project3D(TPoint3D(1.0, 0.0, 0.0), pose);
    EXPECT_NEAR((o - mrpt::math::TPoint3D(1.0, 2.0, 1.0)).norm(), 0.0, 1e-4);
  }
  {
    const auto o = mrpt::math::project3D(
        mrpt::math::TLine3D::FromTwoPoints({0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}), pose);
    EXPECT_TRUE(o.contains({1.0, 1.0, 1.0}));
    EXPECT_TRUE(o.contains({1.0, 2.0, 1.0}));
  }
  {
    const auto o = mrpt::math::project3D(
        mrpt::math::TSegment3D::FromPoints({0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}), pose);
    EXPECT_NEAR((o.point1 - TPoint3D(1.0, 1.0, 1.0)).norm(), 0.0, 1e-5);
    EXPECT_NEAR((o.point2 - TPoint3D(1.0, 2.0, 1.0)).norm(), 0.0, 1e-5);
    EXPECT_TRUE(o.contains({1.0, 2.0, 1.0}));
  }
  {
    const auto o = mrpt::math::project3D(
        mrpt::math::TPolygon3D({
            {0.0, 0.0, 0.0},
            {1.0, 0.0, 0.0},
            {0.0, 1.0, 0.0}
    }),
        pose);
    EXPECT_NEAR((o.at(0) - TPoint3D(1.0, 1.0, 1.0)).norm(), 0.0, 1e-5);
    EXPECT_NEAR((o.at(1) - TPoint3D(1.0, 2.0, 1.0)).norm(), 0.0, 1e-5);
    EXPECT_NEAR((o.at(2) - TPoint3D(0.0, 1.0, 1.0)).norm(), 0.0, 1e-5);
  }
}

TEST(Geometry, getRegressionLine2D)
{
  const std::vector<mrpt::math::TPoint2D> pts = {
      { 0.0,  0.0},
      { 1.0,  0.5},
      { 2.0,  1.0},
      {-4.0, -2.0}
  };

  mrpt::math::TLine2D lin;
  const double err = mrpt::math::getRegressionLine(pts, lin);
  EXPECT_NEAR(err, 0.0, 0.1);

  EXPECT_TRUE(lin.contains({0, 0}));
  EXPECT_TRUE(lin.contains({10.0, 5.0}));
}

TEST(Geometry, getRegressionLine3D)
{
  const std::vector<mrpt::math::TPoint3D> pts = {
      { 0.0,  0.0, 4.0},
      { 1.0,  0.5, 4.0},
      { 2.0,  1.0, 4.0},
      {-4.0, -2.0, 4.0}
  };

  mrpt::math::TLine3D lin;
  const double err = mrpt::math::getRegressionLine(pts, lin);
  EXPECT_NEAR(err, 0.0, 0.1);

  EXPECT_TRUE(lin.contains({0, 0, 4.0}));
  EXPECT_TRUE(lin.contains({10.0, 5.0, 4.0}));
}

TEST(Geometry, getRegressionPlane)
{
  {
    const std::vector<mrpt::math::TPoint3D> pts = {
        { 0.0,   0.0, 0.0},
        { 1.0,   0.5, 0.0},
        {-4.4, -10.0, 0.0},
        { 5.0,  17.0, 0.0}
    };

    mrpt::math::TPlane pl;
    const double err = mrpt::math::getRegressionPlane(pts, pl);
    EXPECT_NEAR(err, 0.0, 1e-3);

    EXPECT_TRUE(pl.contains({0, 0, 0}));
    EXPECT_TRUE(pl.contains({1, 0, 0}));
    EXPECT_TRUE(pl.contains({0, 1, 0}));
  }

  // aligned points should throw:
  {
    const std::vector<mrpt::math::TPoint3D> pts = {
        { 0.0,  0.0, 4.0},
        { 1.0,  0.5, 4.0},
        { 2.0,  1.0, 4.0},
        {-4.0, -2.0, 4.0}
    };

    mrpt::math::TPlane pl;
    EXPECT_ANY_THROW(mrpt::math::getRegressionPlane(pts, pl));
  }
}

// =========================================================================
//  TLine3D::FromPoseX/Y/Z / TLine2D::FromPoseX/Y — lines from pose axes
// =========================================================================

TEST(Geometry, FromPoseX_3D)
{
  // A pose at (1,2,3) with zero rotation: X axis points in world +X direction
  const mrpt::math::TPose3D pose{1.0, 2.0, 3.0, 0.0, 0.0, 0.0};
  const TLine3D r = TLine3D::FromPoseX(pose);

  // Base point must be the pose origin
  EXPECT_NEAR(r.pBase.x, 1.0, 1e-9);
  EXPECT_NEAR(r.pBase.y, 2.0, 1e-9);
  EXPECT_NEAR(r.pBase.z, 3.0, 1e-9);

  // Direction must be unit +X
  EXPECT_NEAR(std::abs(r.director[0]), 1.0, 1e-9);
  EXPECT_NEAR(r.director[1], 0.0, 1e-9);
  EXPECT_NEAR(r.director[2], 0.0, 1e-9);
}

TEST(Geometry, FromPoseY_3D)
{
  const mrpt::math::TPose3D pose{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  const TLine3D r = TLine3D::FromPoseY(pose);

  EXPECT_NEAR(r.director[0], 0.0, 1e-9);
  EXPECT_NEAR(std::abs(r.director[1]), 1.0, 1e-9);
  EXPECT_NEAR(r.director[2], 0.0, 1e-9);
}

TEST(Geometry, FromPoseZ_3D)
{
  const mrpt::math::TPose3D pose{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  const TLine3D r = TLine3D::FromPoseZ(pose);

  EXPECT_NEAR(r.director[0], 0.0, 1e-9);
  EXPECT_NEAR(r.director[1], 0.0, 1e-9);
  EXPECT_NEAR(std::abs(r.director[2]), 1.0, 1e-9);
}

TEST(Geometry, FromPoseX_2D)
{
  // coefs = (cos φ, −sin φ, …): at φ=0 normal is world +X → line is x = 1 (vertical)
  const mrpt::math::TPose2D pose{1.0, 2.0, 0.0};
  const TLine2D r = TLine2D::FromPoseX(pose);

  EXPECT_TRUE(r.contains({1.0, 2.0}));
  EXPECT_TRUE(r.contains({1.0, 5.0}));   // same x, different y
  EXPECT_FALSE(r.contains({3.0, 2.0}));  // different x
}

TEST(Geometry, FromPoseY_2D)
{
  // coefs = (sin φ, cos φ, …): at φ=0 normal is world +Y → line is y = 2 (horizontal)
  const mrpt::math::TPose2D pose{1.0, 2.0, 0.0};
  const TLine2D r = TLine2D::FromPoseY(pose);

  EXPECT_TRUE(r.contains({1.0, 2.0}));
  EXPECT_TRUE(r.contains({3.0, 2.0}));   // same y, different x
  EXPECT_FALSE(r.contains({1.0, 5.0}));  // different y
}

TEST(Geometry, FromPoseDirection_3D)
{
  const mrpt::math::TPose3D pose{1.0, 2.0, 3.0, 0.0, 0.0, 0.0};
  const double v[3] = {0.0, 1.0, 0.0};  // local +Y
  const TLine3D r = TLine3D::FromPoseDirection(pose, v);

  EXPECT_NEAR(r.pBase.x, 1.0, 1e-9);
  EXPECT_NEAR(r.pBase.y, 2.0, 1e-9);
  EXPECT_NEAR(r.pBase.z, 3.0, 1e-9);
  // With zero rotation local +Y maps to world +Y
  EXPECT_NEAR(r.director[0], 0.0, 1e-9);
  EXPECT_NEAR(std::abs(r.director[1]), 1.0, 1e-9);
}

TEST(Geometry, FromPoseDirection_2D)
{
  const mrpt::math::TPose2D pose{1.0, 2.0, 0.0};
  const double v[2] = {0.0, 1.0};  // local +Y maps to world +Y
  const TLine2D r = TLine2D::FromPoseDirection(pose, v);

  // vector (0,1) in local frame at phi=0 → world Y direction → normal is +Y → y=2 (horizontal)
  EXPECT_TRUE(r.contains({1.0, 2.0}));
  EXPECT_TRUE(r.contains({4.0, 2.0}));
}

// =========================================================================
//  createPlaneFromPose*
// =========================================================================

TEST(Geometry, createPlaneFromPoseXY)
{
  // XY plane of identity pose is the world Z=0 plane
  const mrpt::math::TPose3D pose{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  TPlane pl;
  mrpt::math::createPlaneFromPoseXY(pose, pl);

  EXPECT_TRUE(pl.contains({0.0, 0.0, 0.0}));
  EXPECT_TRUE(pl.contains({1.0, 0.0, 0.0}));
  EXPECT_TRUE(pl.contains({0.0, 1.0, 0.0}));
  EXPECT_FALSE(pl.contains({0.0, 0.0, 1.0}));
}

TEST(Geometry, createPlaneFromPoseXZ)
{
  const mrpt::math::TPose3D pose{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  TPlane pl;
  mrpt::math::createPlaneFromPoseXZ(pose, pl);

  EXPECT_TRUE(pl.contains({0.0, 0.0, 0.0}));
  EXPECT_TRUE(pl.contains({1.0, 0.0, 0.0}));
  EXPECT_TRUE(pl.contains({0.0, 0.0, 1.0}));
  EXPECT_FALSE(pl.contains({0.0, 1.0, 0.0}));
}

TEST(Geometry, createPlaneFromPoseYZ)
{
  const mrpt::math::TPose3D pose{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  TPlane pl;
  mrpt::math::createPlaneFromPoseYZ(pose, pl);

  EXPECT_TRUE(pl.contains({0.0, 0.0, 0.0}));
  EXPECT_TRUE(pl.contains({0.0, 1.0, 0.0}));
  EXPECT_TRUE(pl.contains({0.0, 0.0, 1.0}));
  EXPECT_FALSE(pl.contains({1.0, 0.0, 0.0}));
}

// =========================================================================
//  distance() free functions
// =========================================================================

TEST(Geometry, distance_point_point)
{
  EXPECT_NEAR(mrpt::math::distance(TPoint2D(0, 0), TPoint2D(3, 4)), 5.0, 1e-9);
  EXPECT_NEAR(mrpt::math::distance(TPoint3D(0, 0, 0), TPoint3D(1, 2, 2)), 3.0, 1e-9);
}

TEST(Geometry, distance_parallel_lines_2D)
{
  // Two parallel horizontal lines y=0 and y=3
  const TLine2D l1 = TLine2D::FromTwoPoints({0, 0}, {1, 0});
  const TLine2D l2 = TLine2D::FromTwoPoints({0, 3}, {1, 3});
  EXPECT_NEAR(mrpt::math::distance(l1, l2), 3.0, 1e-9);
}

TEST(Geometry, distance_intersecting_lines_2D)
{
  // Intersecting lines: distance is 0
  const TLine2D l1 = TLine2D::FromTwoPoints({0, 0}, {1, 0});
  const TLine2D l2 = TLine2D::FromTwoPoints({0, 0}, {0, 1});
  EXPECT_NEAR(mrpt::math::distance(l1, l2), 0.0, 1e-9);
}

TEST(Geometry, distance_parallel_planes)
{
  const TPlane p1 = TPlane::FromPointAndNormal({0, 0, 0}, {0, 0, 1});
  const TPlane p2 = TPlane::FromPointAndNormal({0, 0, 5}, {0, 0, 1});
  EXPECT_NEAR(mrpt::math::distance(p1, p2), 5.0, 1e-9);
}

TEST(Geometry, distance_intersecting_planes)
{
  const TPlane p1 = TPlane::FromPointAndNormal({0, 0, 0}, {0, 0, 1});
  const TPlane p2 = TPlane::FromPointAndNormal({0, 0, 0}, {0, 1, 0});
  EXPECT_NEAR(mrpt::math::distance(p1, p2), 0.0, 1e-9);
}

TEST(Geometry, distance_parallel_lines_3D)
{
  const auto l1 = TLine3D::FromTwoPoints({0, 0, 0}, {1, 0, 0});
  const auto l2 = TLine3D::FromTwoPoints({0, 2, 0}, {1, 2, 0});
  EXPECT_NEAR(mrpt::math::distance(l1, l2), 2.0, 1e-9);
}

// =========================================================================
//  areAligned — output-parameter variants
// =========================================================================

TEST(Geometry, areAligned_2D_with_line_output)
{
  const std::vector<mrpt::math::TPoint2D> pts = {
      {0, 0},
      {1, 2},
      {2, 4},
      {3, 6}
  };
  TLine2D line;
  EXPECT_TRUE(mrpt::math::areAligned(pts, line));
  // All points must lie on the returned line
  for (const auto& p : pts) EXPECT_TRUE(line.contains(p));
}

TEST(Geometry, areAligned_3D_with_line_output)
{
  const std::vector<mrpt::math::TPoint3D> pts = {
      {0, 0, 0},
      {1, 1, 1},
      {2, 2, 2},
      {3, 3, 3}
  };
  TLine3D line;
  EXPECT_TRUE(mrpt::math::areAligned(pts, line));
  for (const auto& p : pts) EXPECT_TRUE(line.contains(p));
}

// =========================================================================
//  intersect — segment/plane, plane/line, line/line (3D)
// =========================================================================

TEST(Geometry, intersect_segment3D_plane)
{
  // Segment from (0,0,-1) to (0,0,+1) crosses Z=0 plane
  const TSegment3D seg(TPoint3D(0, 0, -1), TPoint3D(0, 0, +1));
  const TPlane plane = TPlane::FromPointAndNormal({0, 0, 0}, {0, 0, 1});

  TObject3D obj;
  EXPECT_TRUE(mrpt::math::intersect(seg, plane, obj));
  EXPECT_TRUE(obj.isPoint());

  TPoint3D pt;
  obj.getPoint(pt);
  EXPECT_NEAR(pt.x, 0.0, 1e-9);
  EXPECT_NEAR(pt.y, 0.0, 1e-9);
  EXPECT_NEAR(pt.z, 0.0, 1e-9);
}

TEST(Geometry, intersect_segment3D_plane_no_crossing)
{
  // Segment lies entirely above Z=0
  const TSegment3D seg(TPoint3D(0, 0, 1), TPoint3D(1, 1, 2));
  const TPlane plane = TPlane::FromPointAndNormal({0, 0, 0}, {0, 0, 1});

  TObject3D obj;
  EXPECT_FALSE(mrpt::math::intersect(seg, plane, obj));
}

TEST(Geometry, intersect_plane_line3D)
{
  // Line along Z axis crosses Z=0 plane at origin
  const TLine3D line = TLine3D::FromTwoPoints({0, 0, -1}, {0, 0, 1});
  const TPlane plane = TPlane::FromPointAndNormal({0, 0, 0}, {0, 0, 1});

  TObject3D obj;
  EXPECT_TRUE(mrpt::math::intersect(plane, line, obj));
  EXPECT_TRUE(obj.isPoint());

  TPoint3D pt;
  obj.getPoint(pt);
  EXPECT_NEAR(pt.z, 0.0, 1e-9);
}

TEST(Geometry, intersect_line3D_line3D_crossing)
{
  // Two coplanar lines that cross at origin
  const TLine3D l1 = TLine3D::FromTwoPoints({-1, 0, 0}, {1, 0, 0});
  const TLine3D l2 = TLine3D::FromTwoPoints({0, -1, 0}, {0, 1, 0});

  TObject3D obj;
  EXPECT_TRUE(mrpt::math::intersect(l1, l2, obj));
  EXPECT_TRUE(obj.isPoint());

  TPoint3D pt;
  obj.getPoint(pt);
  EXPECT_NEAR(pt.x, 0.0, 1e-9);
  EXPECT_NEAR(pt.y, 0.0, 1e-9);
  EXPECT_NEAR(pt.z, 0.0, 1e-9);
}

TEST(Geometry, intersect_line3D_line3D_skew)
{
  // Skew lines (no intersection)
  const TLine3D l1 = TLine3D::FromTwoPoints({0, 0, 0}, {1, 0, 0});
  const TLine3D l2 = TLine3D::FromTwoPoints({0, 1, 1}, {1, 1, 1});

  TObject3D obj;
  EXPECT_FALSE(mrpt::math::intersect(l1, l2, obj));
}

// =========================================================================
//  intersect — polygon2D with segment/line
// =========================================================================

TEST(Geometry, intersect_polygon2D_segment)
{
  // Unit square polygon
  const mrpt::math::TPolygon2D poly = {
      {0, 0},
      {1, 0},
      {1, 1},
      {0, 1}
  };
  // Horizontal segment crossing the square
  const TSegment2D seg(TPoint2D(-0.5, 0.5), TPoint2D(1.5, 0.5));

  TObject2D obj;
  EXPECT_TRUE(mrpt::math::intersect(poly, seg, obj));
}

TEST(Geometry, intersect_polygon2D_line)
{
  const mrpt::math::TPolygon2D poly = {
      {0, 0},
      {1, 0},
      {1, 1},
      {0, 1}
  };
  // Line y=0.5 (horizontal, crosses the square)
  const TLine2D line = TLine2D::FromTwoPoints({-1, 0.5}, {2, 0.5});

  TObject2D obj;
  EXPECT_TRUE(mrpt::math::intersect(poly, line, obj));
}

// =========================================================================
//  project3D — line and plane overloads
// =========================================================================

TEST(Geometry, project3D_line)
{
  // Project the X-axis line through a pose that shifts origin to (0,0,1)
  const TLine3D line = TLine3D::FromTwoPoints({0, 0, 0}, {1, 0, 0});
  const mrpt::math::TPose3D pose{0.0, 0.0, 1.0, 0.0, 0.0, 0.0};

  TLine3D newLine;
  mrpt::math::project3D(line, pose, newLine);

  // Base point must be shifted by (0,0,1)
  EXPECT_NEAR(newLine.pBase.z, 1.0, 1e-9);
  // Direction unchanged (zero rotation)
  EXPECT_NEAR(std::abs(newLine.director[0]), 1.0, 1e-9);
}

TEST(Geometry, project3D_plane)
{
  // Z=0 plane projected through a pose that shifts (0,0,2): gives Z=2 plane
  const TPlane plane = TPlane::FromPointAndNormal({0, 0, 0}, {0, 0, 1});
  const mrpt::math::TPose3D pose{0.0, 0.0, 2.0, 0.0, 0.0, 0.0};

  TPlane newPlane;
  mrpt::math::project3D(plane, pose, newPlane);

  EXPECT_TRUE(newPlane.contains({0, 0, 2}));
  EXPECT_TRUE(newPlane.contains({1, 0, 2}));
  EXPECT_FALSE(newPlane.contains({0, 0, 0}));
}

// =========================================================================
//  signedArea
// =========================================================================

TEST(Geometry, signedArea_CCW_positive)
{
  // Counter-clockwise unit square → positive signed area = 1
  const mrpt::math::TPolygon2D poly = {
      {0, 0},
      {1, 0},
      {1, 1},
      {0, 1}
  };
  EXPECT_NEAR(mrpt::math::signedArea(poly), 1.0, 1e-9);
}

TEST(Geometry, signedArea_CW_negative)
{
  // Clockwise winding → negative
  const mrpt::math::TPolygon2D poly = {
      {0, 0},
      {0, 1},
      {1, 1},
      {1, 0}
  };
  EXPECT_NEAR(mrpt::math::signedArea(poly), -1.0, 1e-9);
}

TEST(Geometry, signedArea_triangle)
{
  // Right triangle with legs 3 and 4: area = 6
  const mrpt::math::TPolygon2D poly = {
      {0, 0},
      {3, 0},
      {0, 4}
  };
  EXPECT_NEAR(std::abs(mrpt::math::signedArea(poly)), 6.0, 1e-9);
}
