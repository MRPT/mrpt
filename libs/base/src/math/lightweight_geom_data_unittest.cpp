/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */


#include <mrpt/base.h>
#include <gtest/gtest.h>

using namespace mrpt;
using namespace mrpt::slam;
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

