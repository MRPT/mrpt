/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/math/CPolygon.h>
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
		TPoint3D(0, 0, 0), TPoint3D(cos(DEG2RAD(30.0)), sin(DEG2RAD(30.0)), 0));
	EXPECT_NEAR(mrpt::RAD2DEG(mrpt::math::getAngle(l1, l4)), 30.0, 1e-5);
	EXPECT_NEAR(mrpt::RAD2DEG(mrpt::math::getAngle(l4, l1)), 30.0, 1e-5);
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
