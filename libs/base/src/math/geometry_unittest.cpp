/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */


#include <mrpt/math/geometry.h>
#include <mrpt/math/CPolygon.h>
#include <gtest/gtest.h>
#include <algorithm>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace std;


TEST(Geometry, Line2DIntersect)
{
	// Two lines that should intersect at (0.5,0.5)
	const TLine2D l1(TPoint2D(0,1), TPoint2D(1,0));
	const TLine2D l2(TPoint2D(-1,0.5), TPoint2D(4,0.5));

	TObject2D inter;
	bool do_inter = intersect(l1, l2, inter);

	EXPECT_TRUE(do_inter);
	EXPECT_EQ( inter.getType(), GEOMETRIC_TYPE_POINT );

	TPoint2D i(0,0);
	inter.getPoint(i);
	EXPECT_NEAR(i.x, 0.5, 1e-9);
	EXPECT_NEAR(i.y, 0.5, 1e-9);
}

TEST(Geometry, Segment2DIntersect)
{
	{
		// Two segments that should intersect at (0.5,0.5)
		const TSegment2D s1(TPoint2D(0,1), TPoint2D(1,0));
		const TSegment2D s2(TPoint2D(-1,0.5), TPoint2D(4,0.5));

		TObject2D inter;
		bool do_inter = intersect(s1, s2, inter);

		EXPECT_TRUE(do_inter);
		EXPECT_EQ( inter.getType(), GEOMETRIC_TYPE_POINT );

		TPoint2D i(0,0);
		inter.getPoint(i);
		EXPECT_NEAR(i.x, 0.5, 1e-9);
		EXPECT_NEAR(i.y, 0.5, 1e-9);
	}

	{
		// Two segments that do NOT intersect
		const TSegment2D s1(TPoint2D(0,1), TPoint2D(1,0));
		const TSegment2D s2(TPoint2D(0.6,0.5), TPoint2D(4,0.5));

		TObject2D inter;
		bool do_inter = intersect(s1, s2, inter);

		EXPECT_FALSE(do_inter);
	}

#if 0
	{
		// Two parallel segments that do NOT intersect: result is a "segment in the middle".
		const TSegment2D s1(TPoint2D(-0.05,0.05), TPoint2D(-0.05,-0.05));
		const TSegment2D s2(TPoint2D(0,0.135), TPoint2D(0,-0.0149999));

		TObject2D inter;
		bool do_inter = intersect(s1, s2, inter);

		EXPECT_TRUE(do_inter && inter.getType()==GEOMETRIC_TYPE_SEGMENT);
	}
#endif
}

void myTestPolygonContainsPoint(std::vector<TPoint2D> &vs, bool convex)
{
	const mrpt::math::TPolygon2D poly(vs);

	EXPECT_EQ(poly.isConvex(),convex);

	EXPECT_TRUE( poly.contains( TPoint2D(0.0, 0.0) ) );
	EXPECT_TRUE( poly.contains( TPoint2D(0.0, 0.9) ) );
	EXPECT_TRUE( poly.contains( TPoint2D(-0.9, -0.9) ) );
	EXPECT_TRUE( poly.contains( TPoint2D(0.9, -0.9) ) );

	EXPECT_FALSE( poly.contains( TPoint2D(-4.0, -5.1) ) );
	EXPECT_FALSE( poly.contains( TPoint2D(-5.0, -0.1) ) );
	EXPECT_FALSE( poly.contains( TPoint2D( 1.1, -6.1) ) );
	EXPECT_FALSE( poly.contains( TPoint2D( 0, 5.1) ) );
	EXPECT_FALSE( poly.contains( TPoint2D( 0, -1.1) ) );
}

TEST(Geometry, PolygonConvexContainsPoint)
{
	// Test with a polygon in one winding order:
	std::vector<TPoint2D> vs;
	vs.push_back(TPoint2D(-1.0, -1.0));
	vs.push_back(TPoint2D( 0.0,  1.0));
	vs.push_back(TPoint2D( 1.0, -1.0));
	myTestPolygonContainsPoint(vs, true);

	// and the other:
	std::reverse(vs.begin(),vs.end());
	myTestPolygonContainsPoint(vs, true);

	{
		mrpt::math::CPolygon p;
		p.AddVertex(0, -0.322);
		p.AddVertex(-0.644, -0.322);
		p.AddVertex(-0.210377, -0.324673);
		p.AddVertex(0.433623, -0.324673);

		EXPECT_FALSE (p.contains(TPoint2D(0.73175, -0.325796)));
	}
}

TEST(Geometry, PolygonConcaveContainsPoint)
{
	// Test with a polygon in one winding order:
	std::vector<TPoint2D> vs;
	vs.push_back(TPoint2D(-2.0,  3.0));
	vs.push_back(TPoint2D( 2.0,  2.0));
	vs.push_back(TPoint2D( 3.0, -4.0));
	vs.push_back(TPoint2D( 0.1, -3.0));
	vs.push_back(TPoint2D( 0.1, -0.1));
	vs.push_back(TPoint2D(-0.1, -0.1));
	vs.push_back(TPoint2D(-0.1, -3.0));
	vs.push_back(TPoint2D(-2.0, -2.0));

	myTestPolygonContainsPoint(vs,false);

	// and the other:
	std::reverse(vs.begin(),vs.end());
	myTestPolygonContainsPoint(vs,false);
}

