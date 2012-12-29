/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                                 |
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
using namespace mrpt::poses;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace std;

// Some 
TEST(SLERP_tests, correctShortestPath)
{
	{	// Both poses at (0,0,0) angles:
		const CPose3D  pose_a(0,0,0, DEG2RAD(0),DEG2RAD(0),DEG2RAD(0));
		const CPose3D  pose_b(0,0,0, DEG2RAD(0),DEG2RAD(0),DEG2RAD(0));
		{
			CPose3D pose_interp;
			mrpt::math::slerp(pose_a,pose_b,0,pose_interp);
			const CPose3D expected(0,0,0,0,0,0);
			EXPECT_NEAR(0, (pose_interp.getHomogeneousMatrixVal()-expected.getHomogeneousMatrixVal()).Abs().sumAll(), 1e-4 ) << "pose_a: " << pose_a << "\npose_b: " << pose_b << "\ninterp: " << pose_interp << endl;
		}
		{
			CPose3D pose_interp;
			mrpt::math::slerp(pose_a,pose_b,1,pose_interp);
			const CPose3D expected(0,0,0,0,0,0);
			EXPECT_NEAR(0, (pose_interp.getHomogeneousMatrixVal()-expected.getHomogeneousMatrixVal()).Abs().sumAll(), 1e-4 ) << "pose_a: " << pose_a << "\npose_b: " << pose_b << "\ninterp: " << pose_interp << endl;
		}
		{
			CPose3D pose_interp;
			mrpt::math::slerp(pose_a,pose_b,0.5,pose_interp);
			const CPose3D expected(0,0,0,0,0,0);
			EXPECT_NEAR(0, (pose_interp.getHomogeneousMatrixVal()-expected.getHomogeneousMatrixVal()).Abs().sumAll(), 1e-4 ) << "pose_a: " << pose_a << "\npose_b: " << pose_b << "\ninterp: " << pose_interp << endl;
		}
	}

	{	// Poses at yaw=+-179deg
		const CPose3D  pose_a(0,0,0, DEG2RAD(179),DEG2RAD(0),DEG2RAD(0));
		const CPose3D  pose_b(0,0,0, DEG2RAD(-179),DEG2RAD(0),DEG2RAD(0));
		CPose3D pose_interp;
		mrpt::math::slerp(pose_a,pose_b,0.5,pose_interp);
		const CPose3D expected(0,0,0,DEG2RAD(-180),0,0);
		EXPECT_NEAR(0, (pose_interp.getHomogeneousMatrixVal()-expected.getHomogeneousMatrixVal()).Abs().sumAll(), 1e-4 ) << "pose_a: " << pose_a << "\npose_b: " << pose_b << "\ninterp: " << pose_interp << endl;
	}
	{	// Poses at yaw=-+179deg
		const CPose3D  pose_a(0,0,0, DEG2RAD(-179),DEG2RAD(0),DEG2RAD(0));
		const CPose3D  pose_b(0,0,0, DEG2RAD(179),DEG2RAD(0),DEG2RAD(0));
		CPose3D pose_interp;
		mrpt::math::slerp(pose_a,pose_b,0.5,pose_interp);
		const CPose3D expected(0,0,0,DEG2RAD(-180),0,0);
		EXPECT_NEAR(0, (pose_interp.getHomogeneousMatrixVal()-expected.getHomogeneousMatrixVal()).Abs().sumAll(), 1e-4 ) << "pose_a: " << pose_a << "\npose_b: " << pose_b << "\ninterp: " << pose_interp << endl;
	}

	{	// Poses at yaw=+-40
		const CPose3D  pose_a(0,0,0, DEG2RAD(40),DEG2RAD(0),DEG2RAD(0));
		const CPose3D  pose_b(0,0,0, DEG2RAD(-40),DEG2RAD(0),DEG2RAD(0));
		CPose3D pose_interp;
		mrpt::math::slerp(pose_a,pose_b,0.5,pose_interp);
		const CPose3D expected(0,0,0,DEG2RAD(0),0,0);
		EXPECT_NEAR(0, (pose_interp.getHomogeneousMatrixVal()-expected.getHomogeneousMatrixVal()).Abs().sumAll(), 1e-4 ) << "pose_a: " << pose_a << "\npose_b: " << pose_b << "\ninterp: " << pose_interp << endl;
	}
	{	// Poses at yaw=-+40
		const CPose3D  pose_a(0,0,0, DEG2RAD(-40),DEG2RAD(0),DEG2RAD(0));
		const CPose3D  pose_b(0,0,0, DEG2RAD(40),DEG2RAD(0),DEG2RAD(0));
		CPose3D pose_interp;
		mrpt::math::slerp(pose_a,pose_b,0.5,pose_interp);
		const CPose3D expected(0,0,0,DEG2RAD(0),0,0);
		EXPECT_NEAR(0, (pose_interp.getHomogeneousMatrixVal()-expected.getHomogeneousMatrixVal()).Abs().sumAll(), 1e-4 ) << "pose_a: " << pose_a << "\npose_b: " << pose_b << "\ninterp: " << pose_interp << endl;
	}

	{	// Poses at pitch=+-40
		const CPose3D  pose_a(0,0,0, DEG2RAD(0),DEG2RAD( 40),DEG2RAD(0));
		const CPose3D  pose_b(0,0,0, DEG2RAD(0),DEG2RAD(-40),DEG2RAD(0));
		CPose3D pose_interp;
		mrpt::math::slerp(pose_a,pose_b,0.5,pose_interp);
		const CPose3D expected(0,0,0,DEG2RAD(0),0,0);
		EXPECT_NEAR(0, (pose_interp.getHomogeneousMatrixVal()-expected.getHomogeneousMatrixVal()).Abs().sumAll(), 1e-4 ) << "pose_a: " << pose_a << "\npose_b: " << pose_b << "\ninterp: " << pose_interp << endl;
	}
	{	// Poses at pitch=-+40
		const CPose3D  pose_a(0,0,0, DEG2RAD(0),DEG2RAD(-40),DEG2RAD(0));
		const CPose3D  pose_b(0,0,0, DEG2RAD(0),DEG2RAD( 40),DEG2RAD(0));
		CPose3D pose_interp;
		mrpt::math::slerp(pose_a,pose_b,0.5,pose_interp);
		const CPose3D expected(0,0,0,DEG2RAD(0),0,0);
		EXPECT_NEAR(0, (pose_interp.getHomogeneousMatrixVal()-expected.getHomogeneousMatrixVal()).Abs().sumAll(), 1e-4 ) << "pose_a: " << pose_a << "\npose_b: " << pose_b << "\ninterp: " << pose_interp << endl;
	}

	{	// Poses at roll=-+40
		const CPose3D  pose_a(0,0,0, DEG2RAD(0),DEG2RAD(0),DEG2RAD(-40));
		const CPose3D  pose_b(0,0,0, DEG2RAD(0),DEG2RAD(0),DEG2RAD( 40));
		CPose3D pose_interp;
		mrpt::math::slerp(pose_a,pose_b,0.5,pose_interp);
		const CPose3D expected(0,0,0,DEG2RAD(0),0,0);
		EXPECT_NEAR(0, (pose_interp.getHomogeneousMatrixVal()-expected.getHomogeneousMatrixVal()).Abs().sumAll(), 1e-4 ) << "pose_a: " << pose_a << "\npose_b: " << pose_b << "\ninterp: " << pose_interp << endl;
	}
	{	// Poses at roll=+-40
		const CPose3D  pose_a(0,0,0, DEG2RAD(0),DEG2RAD(0),DEG2RAD( 40));
		const CPose3D  pose_b(0,0,0, DEG2RAD(0),DEG2RAD(0),DEG2RAD(-40));
		CPose3D pose_interp;
		mrpt::math::slerp(pose_a,pose_b,0.5,pose_interp);
		const CPose3D expected(0,0,0,DEG2RAD(0),0,0);
		EXPECT_NEAR(0, (pose_interp.getHomogeneousMatrixVal()-expected.getHomogeneousMatrixVal()).Abs().sumAll(), 1e-4 ) << "pose_a: " << pose_a << "\npose_b: " << pose_b << "\ninterp: " << pose_interp << endl;
	}


}
