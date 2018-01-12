/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include <mrpt/math/slerp.h>
#include <mrpt/math/lightweight_geom_data.h>
#include <mrpt/math/CMatrixFixedNumeric.h>
#include <gtest/gtest.h>

using namespace mrpt;
using namespace mrpt::math;
using namespace std;

// Some
TEST(SLERP_tests, correctShortestPath)
{
	CMatrixDouble44 HM, HMe;
	{  // Both poses at (0,0,0) angles:
		const TPose3D pose_a(0, 0, 0, DEG2RAD(0), DEG2RAD(0), DEG2RAD(0));
		const TPose3D pose_b(0, 0, 0, DEG2RAD(0), DEG2RAD(0), DEG2RAD(0));
		{
			TPose3D pose_interp;
			mrpt::math::slerp(pose_a, pose_b, 0, pose_interp);
			const TPose3D expected(0, 0, 0, 0, 0, 0);
			pose_interp.getHomogeneousMatrix(HM);
			expected.getHomogeneousMatrix(HMe);
			EXPECT_NEAR(0, (HM - HMe).array().abs().sum(), 1e-4)
				<< "pose_a: " << pose_a << "\npose_b: " << pose_b
				<< "\ninterp: " << pose_interp << endl;
		}
		{
			TPose3D pose_interp;
			mrpt::math::slerp(pose_a, pose_b, 1, pose_interp);
			const TPose3D expected(0, 0, 0, 0, 0, 0);
			pose_interp.getHomogeneousMatrix(HM);
			expected.getHomogeneousMatrix(HMe);
			EXPECT_NEAR(0, (HM - HMe).array().abs().sum(), 1e-4)
				<< "pose_a: " << pose_a << "\npose_b: " << pose_b
				<< "\ninterp: " << pose_interp << endl;
		}
		{
			TPose3D pose_interp;
			mrpt::math::slerp(pose_a, pose_b, 0.5, pose_interp);
			const TPose3D expected(0, 0, 0, 0, 0, 0);
			pose_interp.getHomogeneousMatrix(HM);
			expected.getHomogeneousMatrix(HMe);
			EXPECT_NEAR(0, (HM - HMe).array().abs().sum(), 1e-4)
				<< "pose_a: " << pose_a << "\npose_b: " << pose_b
				<< "\ninterp: " << pose_interp << endl;
		}
	}

	{  // Poses at yaw=+-179deg
		const TPose3D pose_a(0, 0, 0, DEG2RAD(179), DEG2RAD(0), DEG2RAD(0));
		const TPose3D pose_b(0, 0, 0, DEG2RAD(-179), DEG2RAD(0), DEG2RAD(0));
		TPose3D pose_interp;
		mrpt::math::slerp(pose_a, pose_b, 0.5, pose_interp);
		const TPose3D expected(0, 0, 0, DEG2RAD(-180), 0, 0);
		pose_interp.getHomogeneousMatrix(HM);
		expected.getHomogeneousMatrix(HMe);
		EXPECT_NEAR(0, (HM - HMe).array().abs().sum(), 1e-4)
			<< "pose_a: " << pose_a << "\npose_b: " << pose_b
			<< "\ninterp: " << pose_interp << endl;
	}
	{  // Poses at yaw=-+179deg
		const TPose3D pose_a(0, 0, 0, DEG2RAD(-179), DEG2RAD(0), DEG2RAD(0));
		const TPose3D pose_b(0, 0, 0, DEG2RAD(179), DEG2RAD(0), DEG2RAD(0));
		TPose3D pose_interp;
		mrpt::math::slerp(pose_a, pose_b, 0.5, pose_interp);
		const TPose3D expected(0, 0, 0, DEG2RAD(-180), 0, 0);
		pose_interp.getHomogeneousMatrix(HM);
		expected.getHomogeneousMatrix(HMe);
		EXPECT_NEAR(0, (HM - HMe).array().abs().sum(), 1e-4)
			<< "pose_a: " << pose_a << "\npose_b: " << pose_b
			<< "\ninterp: " << pose_interp << endl;
	}
	{  // Poses at yaw=-+179deg
		const TPose3D pose_a(0, 0, 0, DEG2RAD(-179), DEG2RAD(0), DEG2RAD(0));
		const TPose3D pose_b(0, 0, 0, DEG2RAD(179), DEG2RAD(0), DEG2RAD(0));
		TPose3D pose_interp;
		mrpt::math::slerp_ypr(pose_a, pose_b, 0.5, pose_interp);
		const TPose3D expected(0, 0, 0, DEG2RAD(-180), 0, 0);
		pose_interp.getHomogeneousMatrix(HM);
		expected.getHomogeneousMatrix(HMe);
		EXPECT_NEAR(0, (HM - HMe).array().abs().sum(), 1e-4)
			<< "pose_a: " << pose_a.asString()
			<< "\npose_b: " << pose_b.asString()
			<< "\ninterp: " << pose_interp.asString() << endl;
	}

	{  // Poses at yaw=+-40
		const TPose3D pose_a(0, 0, 0, DEG2RAD(40), DEG2RAD(0), DEG2RAD(0));
		const TPose3D pose_b(0, 0, 0, DEG2RAD(-40), DEG2RAD(0), DEG2RAD(0));
		TPose3D pose_interp;
		mrpt::math::slerp(pose_a, pose_b, 0.5, pose_interp);
		const TPose3D expected(0, 0, 0, DEG2RAD(0), 0, 0);
		pose_interp.getHomogeneousMatrix(HM);
		expected.getHomogeneousMatrix(HMe);
		EXPECT_NEAR(0, (HM - HMe).array().abs().sum(), 1e-4)
			<< "pose_a: " << pose_a << "\npose_b: " << pose_b
			<< "\ninterp: " << pose_interp << endl;
	}
	{  // Poses at yaw=-+40
		const TPose3D pose_a(0, 0, 0, DEG2RAD(-40), DEG2RAD(0), DEG2RAD(0));
		const TPose3D pose_b(0, 0, 0, DEG2RAD(40), DEG2RAD(0), DEG2RAD(0));
		TPose3D pose_interp;
		mrpt::math::slerp(pose_a, pose_b, 0.5, pose_interp);
		const TPose3D expected(0, 0, 0, DEG2RAD(0), 0, 0);
		pose_interp.getHomogeneousMatrix(HM);
		expected.getHomogeneousMatrix(HMe);
		EXPECT_NEAR(0, (HM - HMe).array().abs().sum(), 1e-4)
			<< "pose_a: " << pose_a << "\npose_b: " << pose_b
			<< "\ninterp: " << pose_interp << endl;
	}

	{  // Poses at pitch=+-40
		const TPose3D pose_a(0, 0, 0, DEG2RAD(0), DEG2RAD(40), DEG2RAD(0));
		const TPose3D pose_b(0, 0, 0, DEG2RAD(0), DEG2RAD(-40), DEG2RAD(0));
		TPose3D pose_interp;
		mrpt::math::slerp(pose_a, pose_b, 0.5, pose_interp);
		const TPose3D expected(0, 0, 0, DEG2RAD(0), 0, 0);
		pose_interp.getHomogeneousMatrix(HM);
		expected.getHomogeneousMatrix(HMe);
		EXPECT_NEAR(0, (HM - HMe).array().abs().sum(), 1e-4)
			<< "pose_a: " << pose_a << "\npose_b: " << pose_b
			<< "\ninterp: " << pose_interp << endl;
	}
	{  // Poses at pitch=-+40
		const TPose3D pose_a(0, 0, 0, DEG2RAD(0), DEG2RAD(-40), DEG2RAD(0));
		const TPose3D pose_b(0, 0, 0, DEG2RAD(0), DEG2RAD(40), DEG2RAD(0));
		TPose3D pose_interp;
		mrpt::math::slerp(pose_a, pose_b, 0.5, pose_interp);
		const TPose3D expected(0, 0, 0, DEG2RAD(0), 0, 0);
		pose_interp.getHomogeneousMatrix(HM);
		expected.getHomogeneousMatrix(HMe);
		EXPECT_NEAR(0, (HM - HMe).array().abs().sum(), 1e-4)
			<< "pose_a: " << pose_a << "\npose_b: " << pose_b
			<< "\ninterp: " << pose_interp << endl;
	}

	{  // Poses at roll=-+40
		const TPose3D pose_a(0, 0, 0, DEG2RAD(0), DEG2RAD(0), DEG2RAD(-40));
		const TPose3D pose_b(0, 0, 0, DEG2RAD(0), DEG2RAD(0), DEG2RAD(40));
		TPose3D pose_interp;
		mrpt::math::slerp(pose_a, pose_b, 0.5, pose_interp);
		const TPose3D expected(0, 0, 0, DEG2RAD(0), 0, 0);
		pose_interp.getHomogeneousMatrix(HM);
		expected.getHomogeneousMatrix(HMe);
		EXPECT_NEAR(0, (HM - HMe).array().abs().sum(), 1e-4)
			<< "pose_a: " << pose_a << "\npose_b: " << pose_b
			<< "\ninterp: " << pose_interp << endl;
	}
	{  // Poses at roll=+-40
		const TPose3D pose_a(0, 0, 0, DEG2RAD(0), DEG2RAD(0), DEG2RAD(40));
		const TPose3D pose_b(0, 0, 0, DEG2RAD(0), DEG2RAD(0), DEG2RAD(-40));
		TPose3D pose_interp;
		mrpt::math::slerp(pose_a, pose_b, 0.5, pose_interp);
		const TPose3D expected(0, 0, 0, DEG2RAD(0), 0, 0);
		pose_interp.getHomogeneousMatrix(HM);
		expected.getHomogeneousMatrix(HMe);
		EXPECT_NEAR(0, (HM - HMe).array().abs().sum(), 1e-4)
			<< "pose_a: " << pose_a << "\npose_b: " << pose_b
			<< "\ninterp: " << pose_interp << endl;
	}
}
