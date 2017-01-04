/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/nav/planners/nav_plan_geometry_utils.h>
#include <cmath>
#include <gtest/gtest.h>

TEST(NavTests, NavGeomUtils_collision_straight_circ_robot)
{
	using namespace mrpt;
	using namespace mrpt::math;
	using namespace mrpt::nav;

	{
		TPoint2D p0(0,0), p1(1,1);
		double colDist;
		{
			const double R = 0.1;
			TPoint2D pObs(1.0,0.0);
			const bool ret = collision_free_dist_segment_circ_robot(p0, p1, R, pObs, colDist);
			EXPECT_FALSE(ret);
		}
		{
			const double R = 0.1;
			TPoint2D pObs(0.0, 1.0);
			const bool ret = collision_free_dist_segment_circ_robot(p0, p1, R, pObs, colDist);
			EXPECT_FALSE(ret);
		}
		{
			const double R = 0.1;
			TPoint2D pObs(-0.4, -0.4);
			const bool ret = collision_free_dist_segment_circ_robot(p0, p1, R, pObs, colDist);
			EXPECT_FALSE(ret);
		}
		{
			const double R = 0.1;
			TPoint2D pObs(1.4, 1.4);
			const bool ret = collision_free_dist_segment_circ_robot(p0, p1, R, pObs, colDist);
			EXPECT_FALSE(ret);
		}
		{
			const double R = 0.1;
			TPoint2D pObs(0.45, 0.48);
			const bool ret = collision_free_dist_segment_circ_robot(p0, p1, R, pObs, colDist);
			EXPECT_TRUE(ret);
		}
	}

	{
		TPoint2D p0(4, 5), p1(5, 5);
		double colDist;
		const double R = 0.5;
		{
			TPoint2D pObs(5.0, 5.0);
			const bool ret = collision_free_dist_segment_circ_robot(p0, p1, R, pObs, colDist);
			EXPECT_TRUE(ret);
			EXPECT_NEAR(colDist, 0.5, 1e-6);
		}
		{
			TPoint2D pObs(4.510, 5.001);
			const bool ret = collision_free_dist_segment_circ_robot(p0, p1, R, pObs, colDist);
			EXPECT_TRUE(ret);
			EXPECT_NEAR(colDist, 0.01, 3e-3);
		}
	}
	{
		TPoint2D p0(0, 0), p1(-2, -1);
		double colDist;
		const double R = 0.5;
		TPoint2D pObs(-2.0, -1.0);
		const bool ret = collision_free_dist_segment_circ_robot(p0, p1, R, pObs, colDist);
		EXPECT_TRUE(ret);
		EXPECT_NEAR(colDist, std::sqrt(5.0)-0.5, 1e-6);
	}
}
