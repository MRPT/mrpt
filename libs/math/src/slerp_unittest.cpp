/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/slerp.h>

using namespace mrpt;
using namespace mrpt::math;
using namespace std;

// Some
TEST(SLERP_tests, correctShortestPath)
{
	CMatrixDouble44 HM, HMe;
	{  // Both poses at (0,0,0) angles:
		const TPose3D pose_a(0, 0, 0, 0.0_deg, 0.0_deg, 0.0_deg);
		const TPose3D pose_b(0, 0, 0, 0.0_deg, 0.0_deg, 0.0_deg);
		{
			TPose3D pose_interp;
			mrpt::math::slerp(pose_a, pose_b, 0, pose_interp);
			const TPose3D expected(0, 0, 0, 0, 0, 0);
			pose_interp.getHomogeneousMatrix(HM);
			expected.getHomogeneousMatrix(HMe);
			EXPECT_NEAR(0, (HM - HMe).sum_abs(), 1e-4)
				<< "pose_a: " << pose_a << "\npose_b: " << pose_b
				<< "\ninterp: " << pose_interp << endl;
		}
		{
			TPose3D pose_interp;
			mrpt::math::slerp(pose_a, pose_b, 1, pose_interp);
			const TPose3D expected(0, 0, 0, 0, 0, 0);
			pose_interp.getHomogeneousMatrix(HM);
			expected.getHomogeneousMatrix(HMe);
			EXPECT_NEAR(0, (HM - HMe).sum_abs(), 1e-4)
				<< "pose_a: " << pose_a << "\npose_b: " << pose_b
				<< "\ninterp: " << pose_interp << endl;
		}
		{
			TPose3D pose_interp;
			mrpt::math::slerp(pose_a, pose_b, 0.5, pose_interp);
			const TPose3D expected(0, 0, 0, 0, 0, 0);
			pose_interp.getHomogeneousMatrix(HM);
			expected.getHomogeneousMatrix(HMe);
			EXPECT_NEAR(0, (HM - HMe).sum_abs(), 1e-4)
				<< "pose_a: " << pose_a << "\npose_b: " << pose_b
				<< "\ninterp: " << pose_interp << endl;
		}
	}

	{  // Poses at yaw=+-179deg
		const TPose3D pose_a(0, 0, 0, 179.0_deg, 0.0_deg, 0.0_deg);
		const TPose3D pose_b(0, 0, 0, -179.0_deg, 0.0_deg, 0.0_deg);
		TPose3D pose_interp;
		mrpt::math::slerp(pose_a, pose_b, 0.5, pose_interp);
		const TPose3D expected(0, 0, 0, -180.0_deg, 0, 0);
		pose_interp.getHomogeneousMatrix(HM);
		expected.getHomogeneousMatrix(HMe);
		EXPECT_NEAR(0, (HM - HMe).sum_abs(), 1e-4)
			<< "pose_a: " << pose_a << "\npose_b: " << pose_b
			<< "\ninterp: " << pose_interp << endl;
	}
	{  // Poses at yaw=-+179deg
		const TPose3D pose_a(0, 0, 0, -179.0_deg, 0.0_deg, 0.0_deg);
		const TPose3D pose_b(0, 0, 0, 179.0_deg, 0.0_deg, 0.0_deg);
		TPose3D pose_interp;
		mrpt::math::slerp(pose_a, pose_b, 0.5, pose_interp);
		const TPose3D expected(0, 0, 0, -180.0_deg, 0, 0);
		pose_interp.getHomogeneousMatrix(HM);
		expected.getHomogeneousMatrix(HMe);
		EXPECT_NEAR(0, (HM - HMe).sum_abs(), 1e-4)
			<< "pose_a: " << pose_a << "\npose_b: " << pose_b
			<< "\ninterp: " << pose_interp << endl;
	}
	{  // Poses at yaw=-+179deg
		const TPose3D pose_a(0, 0, 0, -179.0_deg, 0.0_deg, 0.0_deg);
		const TPose3D pose_b(0, 0, 0, 179.0_deg, 0.0_deg, 0.0_deg);
		TPose3D pose_interp;
		mrpt::math::slerp_ypr(pose_a, pose_b, 0.5, pose_interp);
		const TPose3D expected(0, 0, 0, -180.0_deg, 0, 0);
		pose_interp.getHomogeneousMatrix(HM);
		expected.getHomogeneousMatrix(HMe);
		EXPECT_NEAR(0, (HM - HMe).sum_abs(), 1e-4)
			<< "pose_a: " << pose_a.asString()
			<< "\npose_b: " << pose_b.asString()
			<< "\ninterp: " << pose_interp.asString() << endl;
	}

	{  // Poses at yaw=+-40
		const TPose3D pose_a(0, 0, 0, 40.0_deg, 0.0_deg, 0.0_deg);
		const TPose3D pose_b(0, 0, 0, -40.0_deg, 0.0_deg, 0.0_deg);
		TPose3D pose_interp;
		mrpt::math::slerp(pose_a, pose_b, 0.5, pose_interp);
		const TPose3D expected(0, 0, 0, 0.0_deg, 0, 0);
		pose_interp.getHomogeneousMatrix(HM);
		expected.getHomogeneousMatrix(HMe);
		EXPECT_NEAR(0, (HM - HMe).sum_abs(), 1e-4)
			<< "pose_a: " << pose_a << "\npose_b: " << pose_b
			<< "\ninterp: " << pose_interp << endl;
	}
	{  // Poses at yaw=-+40
		const TPose3D pose_a(0, 0, 0, -40.0_deg, 0.0_deg, 0.0_deg);
		const TPose3D pose_b(0, 0, 0, 40.0_deg, 0.0_deg, 0.0_deg);
		TPose3D pose_interp;
		mrpt::math::slerp(pose_a, pose_b, 0.5, pose_interp);
		const TPose3D expected(0, 0, 0, 0.0_deg, 0, 0);
		pose_interp.getHomogeneousMatrix(HM);
		expected.getHomogeneousMatrix(HMe);
		EXPECT_NEAR(0, (HM - HMe).sum_abs(), 1e-4)
			<< "pose_a: " << pose_a << "\npose_b: " << pose_b
			<< "\ninterp: " << pose_interp << endl;
	}

	{  // Poses at pitch=+-40
		const TPose3D pose_a(0, 0, 0, 0.0_deg, 40.0_deg, 0.0_deg);
		const TPose3D pose_b(0, 0, 0, 0.0_deg, -40.0_deg, 0.0_deg);
		TPose3D pose_interp;
		mrpt::math::slerp(pose_a, pose_b, 0.5, pose_interp);
		const TPose3D expected(0, 0, 0, 0.0_deg, 0, 0);
		pose_interp.getHomogeneousMatrix(HM);
		expected.getHomogeneousMatrix(HMe);
		EXPECT_NEAR(0, (HM - HMe).sum_abs(), 1e-4)
			<< "pose_a: " << pose_a << "\npose_b: " << pose_b
			<< "\ninterp: " << pose_interp << endl;
	}
	{  // Poses at pitch=-+40
		const TPose3D pose_a(0, 0, 0, 0.0_deg, -40.0_deg, 0.0_deg);
		const TPose3D pose_b(0, 0, 0, 0.0_deg, 40.0_deg, 0.0_deg);
		TPose3D pose_interp;
		mrpt::math::slerp(pose_a, pose_b, 0.5, pose_interp);
		const TPose3D expected(0, 0, 0, 0.0_deg, 0, 0);
		pose_interp.getHomogeneousMatrix(HM);
		expected.getHomogeneousMatrix(HMe);
		EXPECT_NEAR(0, (HM - HMe).sum_abs(), 1e-4)
			<< "pose_a: " << pose_a << "\npose_b: " << pose_b
			<< "\ninterp: " << pose_interp << endl;
	}

	{  // Poses at roll=-+40
		const TPose3D pose_a(0, 0, 0, 0.0_deg, 0.0_deg, -40.0_deg);
		const TPose3D pose_b(0, 0, 0, 0.0_deg, 0.0_deg, 40.0_deg);
		TPose3D pose_interp;
		mrpt::math::slerp(pose_a, pose_b, 0.5, pose_interp);
		const TPose3D expected(0, 0, 0, 0.0_deg, 0, 0);
		pose_interp.getHomogeneousMatrix(HM);
		expected.getHomogeneousMatrix(HMe);
		EXPECT_NEAR(0, (HM - HMe).sum_abs(), 1e-4)
			<< "pose_a: " << pose_a << "\npose_b: " << pose_b
			<< "\ninterp: " << pose_interp << endl;
	}
	{  // Poses at roll=+-40
		const TPose3D pose_a(0, 0, 0, 0.0_deg, 0.0_deg, 40.0_deg);
		const TPose3D pose_b(0, 0, 0, 0.0_deg, 0.0_deg, -40.0_deg);
		TPose3D pose_interp;
		mrpt::math::slerp(pose_a, pose_b, 0.5, pose_interp);
		const TPose3D expected(0, 0, 0, 0.0_deg, 0, 0);
		pose_interp.getHomogeneousMatrix(HM);
		expected.getHomogeneousMatrix(HMe);
		EXPECT_NEAR(0, (HM - HMe).sum_abs(), 1e-4)
			<< "pose_a: " << pose_a << "\npose_b: " << pose_b
			<< "\ninterp: " << pose_interp << endl;
	}
}
