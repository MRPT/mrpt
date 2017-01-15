/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */


#include <mrpt/math/lightweight_geom_data.h>
#include <gtest/gtest.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace std;


TEST(LightGeomData, PragmaPack)
{
	{
		TPoint2D p;
		EXPECT_TRUE(&p.x==&(p[0]));
		EXPECT_TRUE(&p.y==&(p[1]));
	}
	{
		TPoint3D p;
		EXPECT_TRUE(&p.x==&(p[0]));
		EXPECT_TRUE(&p.y==&(p[1]));
		EXPECT_TRUE(&p.z==&(p[2]));
	}
	{
		TPose2D p;
		EXPECT_TRUE(&p.x==&(p[0]));
		EXPECT_TRUE(&p.y==&(p[1]));
		EXPECT_TRUE(&p.phi==&(p[2]));
	}
	{
		TPose3D p;
		EXPECT_TRUE(&p.x==&(p[0]));
		EXPECT_TRUE(&p.y==&(p[1]));
		EXPECT_TRUE(&p.z==&(p[2]));
		EXPECT_TRUE(&p.yaw==&(p[3]));
		EXPECT_TRUE(&p.pitch==&(p[4]));
		EXPECT_TRUE(&p.roll==&(p[5]));
	}
	{
		TSegment2D s;
		EXPECT_TRUE(&s.point1==&(s[0]));
		EXPECT_TRUE(&s.point2==&(s[1]));
	}
}

TEST(LightGeomData, ExpectedMemorySizes)
{
	EXPECT_EQ(sizeof(TPoint2D),2*sizeof(double));
	EXPECT_EQ(sizeof(TPoint3D),3*sizeof(double));
	EXPECT_EQ(sizeof(TPoint3Df),3*sizeof(float));
	EXPECT_EQ(sizeof(TPose2D),3*sizeof(double));
	EXPECT_EQ(sizeof(TPose3D),6*sizeof(double));
	EXPECT_EQ(sizeof(TPose3DQuat),7*sizeof(double));
}

