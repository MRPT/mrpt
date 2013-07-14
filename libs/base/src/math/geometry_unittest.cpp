/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
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

	TPoint2D i;
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

		TPoint2D i;
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
