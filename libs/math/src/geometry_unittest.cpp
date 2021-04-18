/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/math/CPolygon.h>
#include <mrpt/math/TLine3D.h>
#include <mrpt/math/TObject2D.h>
#include <mrpt/math/TObject3D.h>
#include <mrpt/math/TPose2D.h>
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
	EXPECT_EQ(inter.getType(), GEOMETRIC_TYPE_POINT);

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

	const TLine3D l4(
		TPoint3D(0, 0, 0), TPoint3D(cos(30.0_deg), sin(30.0_deg), 0));
	EXPECT_NEAR(mrpt::RAD2DEG(mrpt::math::getAngle(l1, l4)), 30.0, 1e-5);
	EXPECT_NEAR(mrpt::RAD2DEG(mrpt::math::getAngle(l4, l1)), 30.0, 1e-5);
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

TEST(Geometry, Segment2DIntersect)
{
	{
		// Two segments that should intersect at (0.5,0.5)
		const TSegment2D s1(TPoint2D(0, 1), TPoint2D(1, 0));
		const TSegment2D s2(TPoint2D(-1, 0.5), TPoint2D(4, 0.5));

		TObject2D inter;
		bool do_inter = intersect(s1, s2, inter);

		EXPECT_TRUE(do_inter);
		EXPECT_EQ(inter.getType(), GEOMETRIC_TYPE_POINT);

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

		// EXPECT_TRUE(do_inter && inter.getType()==GEOMETRIC_TYPE_SEGMENT);
		EXPECT_FALSE(do_inter);
	}
}

TEST(Geometry, Intersection3D)
{
	{
		TPolygon3D p3d({{1, 0, 0}, {0, 1, 0}, {0, 0, 1}});
		TSegment3D s3d({
			{1, 0, 0},
			{0, 1, 0},
		});

		TObject3D inter;
		EXPECT_TRUE(intersect(p3d, s3d, inter));
		EXPECT_EQ(inter.getType(), GEOMETRIC_TYPE_SEGMENT);
		TSegment3D test;
		inter.getSegment(test);
		// Should this be true? EXPECT_EQ(s3d, test);
	}
	{
		TPolygon3D p3d({{1, 0, 0}, {0, 1, 0}, {0, 0, 1}});
		TSegment3D s3d({
			{0, 0, 0},
			{1, 1, 1},
		});

		TObject3D inter;
		EXPECT_TRUE(intersect(p3d, s3d, inter));
		EXPECT_EQ(inter.getType(), GEOMETRIC_TYPE_POINT);
	}
	{
		TSegment3D s3d1({
			{1, 0, 0},
			{0, 1, 0},
		});
		TSegment3D s3d2({
			{2, -1.0, 0},
			{0, 1.0, 0},
		});

		TObject3D inter;
		EXPECT_TRUE(intersect(s3d1, s3d2, inter));
		EXPECT_EQ(inter.getType(), GEOMETRIC_TYPE_SEGMENT);
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
		EXPECT_EQ(inter.getType(), GEOMETRIC_TYPE_POINT);

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
			{-1, 1, 1},
			{1, -1, 1},
			{1, 1, -1},
		});

		TObject3D inter;
		EXPECT_TRUE(intersect(plane1, plane2, inter));
		EXPECT_EQ(inter.getType(), GEOMETRIC_TYPE_PLANE);
	}
	{
		// Intersecting planes
		TPlane plane1({
			{1, 0, 0},
			{0, 1, 0},
			{0, 0, 1},
		});
		TPlane plane2({
			{1, 0, 0},
			{0, -1, 0},
			{0, 0, -1},
		});

		TObject3D inter;
		EXPECT_TRUE(intersect(plane1, plane2, inter));
		EXPECT_EQ(inter.getType(), GEOMETRIC_TYPE_LINE);
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
		p.AddVertex(0, -0.322);
		p.AddVertex(-0.644, -0.322);
		p.AddVertex(-0.210377, -0.324673);
		p.AddVertex(0.433623, -0.324673);

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
			{0., 0., 0.}, {1., 0., 0.}, {1., 1., 0.}, {0., 1., 0.}};
		EXPECT_TRUE(mrpt::math::conformAPlane(pts));
	}
	{
		std::vector<TPoint3D> pts = {
			{0., 0., 0.}, {0., 0., 1.}, {0., 1., 1.}, {0., 1., 0.}};
		EXPECT_TRUE(mrpt::math::conformAPlane(pts));
	}
	{
		std::vector<TPoint3D> pts = {
			{0., 0., 0.}, {0., 0., 1.}, {0., 1., 1.}, {0.1, 1., 0.1}};
		EXPECT_FALSE(mrpt::math::conformAPlane(pts));
	}
	{
		std::vector<TPoint3D> pts = {
			{5.56496063, -2.30508217, 29.53900000},
			{5.87949871, 0.00000000, 29.53900000},
			{13.50000000, 0.00000000, 0.00000000},
			{12.50465807, -7.29433126, 0.00000000}};
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
		{0, 0, 0.0_deg, /*result*/ 1},	  {3.1, 0, 0.0_deg, /*result*/ 0},
		{-3.1, 0, 0.0_deg, /*result*/ 0}, {2.9, 0, 0.0_deg, /*result*/ 1},
		{-2.9, 0, 0.0_deg, /*result*/ 1}, {0, 4.1, 0.0_deg, /*result*/ 0},
		{0, 3.9, 0.0_deg, /*result*/ 1},  {0, -4.1, 0.0_deg, /*result*/ 0},
		{0, -3.9, 0.0_deg, /*result*/ 1}, {3.1, 0, 0.0_deg, /*result*/ 0},
		{3.1, 0, 45.0_deg, /*result*/ 1}, {3.1, 0, -90.0_deg, /*result*/ 1}};

	for (const auto& t : tsts)
	{
		const auto p = mrpt::math::TPose2D(t[0], t[1], t[2]);
		EXPECT_EQ(
			RectanglesIntersection(
				r1_xmin, r1_xmax, r1_ymin, r1_ymax, r2_xmin, r2_xmax, r2_ymin,
				r2_ymax, p.x, p.y, p.phi),
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
		(closestFromPointToSegment({0, 0}, {0, 1}, {1, 1}) - TPoint2D(0, 1))
			.norm(),
		.0, 1e-5);
	EXPECT_NEAR(
		(closestFromPointToSegment({0, 1}, {0, 1}, {1, 1}) - TPoint2D(0, 1))
			.norm(),
		.0, 1e-5);
	EXPECT_NEAR(
		(closestFromPointToSegment({1, 1}, {0, 1}, {1, 1}) - TPoint2D(1, 1))
			.norm(),
		.0, 1e-5);
	EXPECT_NEAR(
		(closestFromPointToSegment({-1, 2}, {0, 1}, {1, 1}) - TPoint2D(0, 1))
			.norm(),
		.0, 1e-5);
	EXPECT_NEAR(
		(closestFromPointToSegment({3, 3}, {0, 1}, {1, 1}) - TPoint2D(1, 1))
			.norm(),
		.0, 1e-5);
	EXPECT_NEAR(
		(closestFromPointToSegment({0.5, 0.1}, {0, 1}, {1, 1}) -
		 TPoint2D(0.5, 1))
			.norm(),
		.0, 1e-5);
}

TEST(Geometry, closestFromPointToLine)
{
	using mrpt::math::closestFromPointToLine;
	using mrpt::math::TPoint2D;

	EXPECT_NEAR(
		(closestFromPointToLine({0, 0}, {0, 1}, {1, 1}) - TPoint2D(0, 1))
			.norm(),
		.0, 1e-5);
	EXPECT_NEAR(
		(closestFromPointToLine({0, 1}, {0, 1}, {1, 1}) - TPoint2D(0, 1))
			.norm(),
		.0, 1e-5);
	EXPECT_NEAR(
		(closestFromPointToLine({1, 1}, {0, 1}, {1, 1}) - TPoint2D(1, 1))
			.norm(),
		.0, 1e-5);
	EXPECT_NEAR(
		(closestFromPointToLine({-1, 2}, {0, 1}, {1, 1}) - TPoint2D(-1, 1))
			.norm(),
		.0, 1e-5);
	EXPECT_NEAR(
		(closestFromPointToLine({3, 3}, {0, 1}, {1, 1}) - TPoint2D(3, 1))
			.norm(),
		.0, 1e-5);
	EXPECT_NEAR(
		(closestFromPointToLine({0.5, 0.1}, {0, 1}, {1, 1}) - TPoint2D(0.5, 1))
			.norm(),
		.0, 1e-5);
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
	EXPECT_NEAR(
		squaredDistancePointToLine({0.5, 0.1}, {0, 1}, {1, 1}), 0.9 * 0.9,
		1e-5);
}
