/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */


#include <mrpt/math/slerp.h>
#include <mrpt/poses/CPose3D.h>
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
			EXPECT_NEAR(0, (pose_interp.getHomogeneousMatrixVal()-expected.getHomogeneousMatrixVal()).array().abs().sum(), 1e-4 ) << "pose_a: " << pose_a << "\npose_b: " << pose_b << "\ninterp: " << pose_interp << endl;
		}
		{
			CPose3D pose_interp;
			mrpt::math::slerp(pose_a,pose_b,1,pose_interp);
			const CPose3D expected(0,0,0,0,0,0);
			EXPECT_NEAR(0, (pose_interp.getHomogeneousMatrixVal()-expected.getHomogeneousMatrixVal()).array().abs().sum(), 1e-4 ) << "pose_a: " << pose_a << "\npose_b: " << pose_b << "\ninterp: " << pose_interp << endl;
		}
		{
			CPose3D pose_interp;
			mrpt::math::slerp(pose_a,pose_b,0.5,pose_interp);
			const CPose3D expected(0,0,0,0,0,0);
			EXPECT_NEAR(0, (pose_interp.getHomogeneousMatrixVal()-expected.getHomogeneousMatrixVal()).array().abs().sum(), 1e-4 ) << "pose_a: " << pose_a << "\npose_b: " << pose_b << "\ninterp: " << pose_interp << endl;
		}
	}

	{	// Poses at yaw=+-179deg
		const CPose3D  pose_a(0,0,0, DEG2RAD(179),DEG2RAD(0),DEG2RAD(0));
		const CPose3D  pose_b(0,0,0, DEG2RAD(-179),DEG2RAD(0),DEG2RAD(0));
		CPose3D pose_interp;
		mrpt::math::slerp(pose_a,pose_b,0.5,pose_interp);
		const CPose3D expected(0,0,0,DEG2RAD(-180),0,0);
		EXPECT_NEAR(0, (pose_interp.getHomogeneousMatrixVal()-expected.getHomogeneousMatrixVal()).array().abs().sum(), 1e-4 ) << "pose_a: " << pose_a << "\npose_b: " << pose_b << "\ninterp: " << pose_interp << endl;
	}
	{	// Poses at yaw=-+179deg
		const CPose3D  pose_a(0,0,0, DEG2RAD(-179),DEG2RAD(0),DEG2RAD(0));
		const CPose3D  pose_b(0,0,0, DEG2RAD(179),DEG2RAD(0),DEG2RAD(0));
		CPose3D pose_interp;
		mrpt::math::slerp(pose_a,pose_b,0.5,pose_interp);
		const CPose3D expected(0,0,0,DEG2RAD(-180),0,0);
		EXPECT_NEAR(0, (pose_interp.getHomogeneousMatrixVal()-expected.getHomogeneousMatrixVal()).array().abs().sum(), 1e-4 ) << "pose_a: " << pose_a << "\npose_b: " << pose_b << "\ninterp: " << pose_interp << endl;
	}

	{	// Poses at yaw=+-40
		const CPose3D  pose_a(0,0,0, DEG2RAD(40),DEG2RAD(0),DEG2RAD(0));
		const CPose3D  pose_b(0,0,0, DEG2RAD(-40),DEG2RAD(0),DEG2RAD(0));
		CPose3D pose_interp;
		mrpt::math::slerp(pose_a,pose_b,0.5,pose_interp);
		const CPose3D expected(0,0,0,DEG2RAD(0),0,0);
		EXPECT_NEAR(0, (pose_interp.getHomogeneousMatrixVal()-expected.getHomogeneousMatrixVal()).array().abs().sum(), 1e-4 ) << "pose_a: " << pose_a << "\npose_b: " << pose_b << "\ninterp: " << pose_interp << endl;
	}
	{	// Poses at yaw=-+40
		const CPose3D  pose_a(0,0,0, DEG2RAD(-40),DEG2RAD(0),DEG2RAD(0));
		const CPose3D  pose_b(0,0,0, DEG2RAD(40),DEG2RAD(0),DEG2RAD(0));
		CPose3D pose_interp;
		mrpt::math::slerp(pose_a,pose_b,0.5,pose_interp);
		const CPose3D expected(0,0,0,DEG2RAD(0),0,0);
		EXPECT_NEAR(0, (pose_interp.getHomogeneousMatrixVal()-expected.getHomogeneousMatrixVal()).array().abs().sum(), 1e-4 ) << "pose_a: " << pose_a << "\npose_b: " << pose_b << "\ninterp: " << pose_interp << endl;
	}

	{	// Poses at pitch=+-40
		const CPose3D  pose_a(0,0,0, DEG2RAD(0),DEG2RAD( 40),DEG2RAD(0));
		const CPose3D  pose_b(0,0,0, DEG2RAD(0),DEG2RAD(-40),DEG2RAD(0));
		CPose3D pose_interp;
		mrpt::math::slerp(pose_a,pose_b,0.5,pose_interp);
		const CPose3D expected(0,0,0,DEG2RAD(0),0,0);
		EXPECT_NEAR(0, (pose_interp.getHomogeneousMatrixVal()-expected.getHomogeneousMatrixVal()).array().abs().sum(), 1e-4 ) << "pose_a: " << pose_a << "\npose_b: " << pose_b << "\ninterp: " << pose_interp << endl;
	}
	{	// Poses at pitch=-+40
		const CPose3D  pose_a(0,0,0, DEG2RAD(0),DEG2RAD(-40),DEG2RAD(0));
		const CPose3D  pose_b(0,0,0, DEG2RAD(0),DEG2RAD( 40),DEG2RAD(0));
		CPose3D pose_interp;
		mrpt::math::slerp(pose_a,pose_b,0.5,pose_interp);
		const CPose3D expected(0,0,0,DEG2RAD(0),0,0);
		EXPECT_NEAR(0, (pose_interp.getHomogeneousMatrixVal()-expected.getHomogeneousMatrixVal()).array().abs().sum(), 1e-4 ) << "pose_a: " << pose_a << "\npose_b: " << pose_b << "\ninterp: " << pose_interp << endl;
	}

	{	// Poses at roll=-+40
		const CPose3D  pose_a(0,0,0, DEG2RAD(0),DEG2RAD(0),DEG2RAD(-40));
		const CPose3D  pose_b(0,0,0, DEG2RAD(0),DEG2RAD(0),DEG2RAD( 40));
		CPose3D pose_interp;
		mrpt::math::slerp(pose_a,pose_b,0.5,pose_interp);
		const CPose3D expected(0,0,0,DEG2RAD(0),0,0);
		EXPECT_NEAR(0, (pose_interp.getHomogeneousMatrixVal()-expected.getHomogeneousMatrixVal()).array().abs().sum(), 1e-4 ) << "pose_a: " << pose_a << "\npose_b: " << pose_b << "\ninterp: " << pose_interp << endl;
	}
	{	// Poses at roll=+-40
		const CPose3D  pose_a(0,0,0, DEG2RAD(0),DEG2RAD(0),DEG2RAD( 40));
		const CPose3D  pose_b(0,0,0, DEG2RAD(0),DEG2RAD(0),DEG2RAD(-40));
		CPose3D pose_interp;
		mrpt::math::slerp(pose_a,pose_b,0.5,pose_interp);
		const CPose3D expected(0,0,0,DEG2RAD(0),0,0);
		EXPECT_NEAR(0, (pose_interp.getHomogeneousMatrixVal()-expected.getHomogeneousMatrixVal()).array().abs().sum(), 1e-4 ) << "pose_a: " << pose_a << "\npose_b: " << pose_b << "\ninterp: " << pose_interp << endl;
	}


}
