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
