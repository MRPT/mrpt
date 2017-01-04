/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */


#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/maps/CWeightedPointsMap.h>
#include <mrpt/maps/CColouredPointsMap.h>
#include <mrpt/poses/CPoint2D.h>
#include <gtest/gtest.h>

using namespace mrpt;
using namespace mrpt::maps;
using namespace mrpt::obs;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace std;

const size_t demo9_N = 9;
const float demo9_xs[demo9_N]={0,0,0,1,1,1,2,2,2};
const float demo9_ys[demo9_N]={0,1,2,0,1,2,0,1,2};
const float demo9_zs[demo9_N]={0,1,2,0,1,2,0,1,2};


template <class MAP>
void load_demo_9pts_map(MAP &pts)
{
	pts.clear();
	for (size_t i=0;i<demo9_N;i++)
		pts.insertPoint(demo9_xs[i],demo9_ys[i],demo9_zs[i]);
}

template <class MAP>
void do_test_insertPoints()
{
	// test 1: Insert and check expected values:
	{
		MAP  pts;
		load_demo_9pts_map(pts);

		EXPECT_EQ(pts.size(),demo9_N);

		for (size_t i=0;i<demo9_N;i++)
		{
			float x,y,z;
			pts.getPoint(i,x,y,z);
			EXPECT_EQ(x,demo9_xs[i]);
			EXPECT_EQ(y,demo9_ys[i]);
			EXPECT_EQ(z,demo9_zs[i]);
		}
	}

	// test 2: Copy between maps
	{
		MAP  pts1;
		load_demo_9pts_map(pts1);

		MAP  pts2 = pts1;
		MAP  pts3 = pts1;

		EXPECT_EQ(pts2.size(),pts3.size());
		for (size_t i=0;i<demo9_N;i++)
		{
			float x2,y2,z2;
			float x3,y3,z3;
			pts2.getPoint(i,x2,y2,z2);
			pts3.getPoint(i,x3,y3,z3);
			EXPECT_EQ(x2,x3);
			EXPECT_EQ(y2,y3);
			EXPECT_EQ(z2,z3);
		}
	}

}

template <class MAP>
void do_test_clipOutOfRangeInZ()
{
	MAP  pts0;
	load_demo_9pts_map(pts0);

	// Clip: z=[-10,-1] -> 0 pts
	{
		MAP  pts = pts0;
		pts.clipOutOfRangeInZ(-10,-1);
		EXPECT_EQ(pts.size(),0u);
	}

	// Clip: z=[-10,10] -> 9 pts
	{
		MAP  pts = pts0;
		pts.clipOutOfRangeInZ(-10,10);
		EXPECT_EQ(pts.size(),9u);
	}

	// Clip: z=[0.5,1.5] -> 3 pts
	{
		MAP  pts = pts0;
		pts.clipOutOfRangeInZ(0.5,1.5);
		EXPECT_EQ(pts.size(),3u);
	}

}

template <class MAP>
void do_test_clipOutOfRange()
{
	MAP  pts0;
	load_demo_9pts_map(pts0);

	// Clip:
	{
		CPoint2D pivot(0,0);
		const float radius = 0.5;

		MAP  pts = pts0;
		pts.clipOutOfRange(pivot,radius);
		EXPECT_EQ(pts.size(),1u);
	}

	// Clip:
	{
		CPoint2D pivot(-10,-10);
		const float radius = 1;

		MAP  pts = pts0;
		pts.clipOutOfRange(pivot,radius);
		EXPECT_EQ(pts.size(),0u);
	}

	// Clip:
	{
		CPoint2D pivot(0,0);
		const float radius = 1.1f;

		MAP  pts = pts0;
		pts.clipOutOfRange(pivot,radius);
		EXPECT_EQ(pts.size(),3u);
	}

}


TEST(CSimplePointsMapTests, insertPoints)
{
	do_test_insertPoints<CSimplePointsMap>();
}

TEST(CWeightedPointsMapTests, insertPoints)
{
	do_test_insertPoints<CWeightedPointsMap>();
}

TEST(CColouredPointsMapTests, insertPoints)
{
	do_test_insertPoints<CColouredPointsMap>();
}


TEST(CSimplePointsMapTests, clipOutOfRangeInZ)
{
	do_test_clipOutOfRangeInZ<CSimplePointsMap>();
}

TEST(CWeightedPointsMapTests, clipOutOfRangeInZ)
{
	do_test_clipOutOfRangeInZ<CWeightedPointsMap>();
}

TEST(CColouredPointsMapTests, clipOutOfRangeInZ)
{
	do_test_clipOutOfRangeInZ<CColouredPointsMap>();
}

TEST(CSimplePointsMapTests, clipOutOfRange)
{
	do_test_clipOutOfRange<CSimplePointsMap>();
}

TEST(CWeightedPointsMapTests, clipOutOfRange)
{
	do_test_clipOutOfRange<CWeightedPointsMap>();
}

TEST(CColouredPointsMapTests, clipOutOfRange)
{
	do_test_clipOutOfRange<CColouredPointsMap>();
}

