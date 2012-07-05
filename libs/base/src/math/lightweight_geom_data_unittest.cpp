/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
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

