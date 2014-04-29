/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */


#include <mrpt/math/geometry.h>
#include <gtest/gtest.h>

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

	{
		// Two parallel segments that do NOT intersect: result is a "segment in the middle".
		const TSegment2D s1(TPoint2D(-0.05,0.05), TPoint2D(-0.05,-0.05));
		const TSegment2D s2(TPoint2D(0,0.135), TPoint2D(0,-0.0149999));

		TObject2D inter;
		bool do_inter = intersect(s1, s2, inter);

		EXPECT_TRUE(do_inter && inter.getType()==GEOMETRIC_TYPE_SEGMENT);
	}
}

void myTestPolygonContainsPoint(std::vector<TPoint2D> &vs)
{
	const mrpt::math::TPolygon2D poly(vs);

	EXPECT_TRUE( poly.contains( TPoint2D(0.0, 0.0) ) );
	EXPECT_TRUE( poly.contains( TPoint2D(0.0, 0.9) ) );
	EXPECT_TRUE( poly.contains( TPoint2D(-0.9, -0.9) ) );
	EXPECT_TRUE( poly.contains( TPoint2D(0.9, -0.9) ) );

	EXPECT_FALSE( poly.contains( TPoint2D(-1.1, -1.1) ) );
	EXPECT_FALSE( poly.contains( TPoint2D( 1.1, -1.1) ) );
	EXPECT_FALSE( poly.contains( TPoint2D( 0, 1.1) ) );
	EXPECT_FALSE( poly.contains( TPoint2D( 0, -1.1) ) );
}

TEST(Geometry, PolygonContainsPoint)
{
	// Test with a polygon in one winding order:
	{
		std::vector<TPoint2D> vs;
		vs.push_back(TPoint2D(-1.0, -1.0));
		vs.push_back(TPoint2D( 0.0,  1.0));
		vs.push_back(TPoint2D( 1.0, -1.0));
		myTestPolygonContainsPoint(vs);
	}
	// and the other:
	{
		std::vector<TPoint2D> vs;
		vs.push_back(TPoint2D( 1.0, -1.0));
		vs.push_back(TPoint2D( 0.0,  1.0));
		vs.push_back(TPoint2D(-1.0, -1.0));
		myTestPolygonContainsPoint(vs);
	}
}

