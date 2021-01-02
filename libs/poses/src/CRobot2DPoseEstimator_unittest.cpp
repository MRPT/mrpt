/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/poses/CRobot2DPoseEstimator.h>
#include <mrpt/system/datetime.h>

TEST(CRobot2DPoseEstimator, defaultCtor)
{
	mrpt::poses::CRobot2DPoseEstimator rpe;

	mrpt::math::TPose2D p;
	mrpt::math::TTwist2D vl, vg;

	bool ok = rpe.getCurrentEstimate(p, vl, vg);

	EXPECT_FALSE(ok);
}

TEST(CRobot2DPoseEstimator, extrapolateRobotPose)
{
	mrpt::math::TPose2D p = {0, 0, 0};
	const mrpt::math::TTwist2D vl = {2.0, 1.0, -0.5};
	const double dt = 2.0;

	mrpt::math::TPose2D new_p;
	mrpt::poses::CRobot2DPoseEstimator::extrapolateRobotPose(p, vl, dt, new_p);

	EXPECT_DOUBLE_EQ(new_p.x, p.x + dt * vl.vx);
	EXPECT_DOUBLE_EQ(new_p.y, p.y + dt * vl.vy);
	EXPECT_DOUBLE_EQ(new_p.phi, p.phi + dt * vl.omega);

	// now, rotated 90deg:
	p = mrpt::math::TPose2D{10.0, 20.0, mrpt::DEG2RAD(90.0)};
	mrpt::poses::CRobot2DPoseEstimator::extrapolateRobotPose(p, vl, dt, new_p);

	EXPECT_NEAR(new_p.x, p.x - dt * vl.vy, 1e-5);
	EXPECT_NEAR(new_p.y, p.y + dt * vl.vx, 1e-5);
	EXPECT_NEAR(new_p.phi, p.phi + dt * vl.omega, 1e-5);
}

TEST(CRobot2DPoseEstimator, integrateOdometryAndLocalization)
{
	const mrpt::math::TPose2D odo1 = {0, 0, 0};
	const mrpt::math::TPose2D loc1 = {10.0, 20.0, mrpt::DEG2RAD(90.0)};
	const mrpt::math::TTwist2D vel1 = {1.0, 0.0, mrpt::DEG2RAD(10.0)};
	const double dt = 1.0;
	const auto odo2 = odo1 + vel1 * dt;

	mrpt::poses::CRobot2DPoseEstimator rpe;

	rpe.params.max_localiz_age = 2 * dt;
	rpe.params.max_odometry_age = 2 * dt;

	const auto t0 = mrpt::Clock::now();
	const auto t1 = mrpt::system::timestampAdd(t0, dt);

	// t0:
	rpe.processUpdateNewOdometry(odo1, t0, true, vel1);
	rpe.processUpdateNewPoseLocalization(loc1, t0);

	mrpt::math::TPose2D p;
	mrpt::math::TTwist2D vl, vg;
	bool ok = rpe.getCurrentEstimate(p, vl, vg, t0);
	EXPECT_TRUE(ok);
	EXPECT_NEAR(p.x, loc1.x, 1e-5);
	EXPECT_NEAR(p.y, loc1.y, 1e-5);
	EXPECT_NEAR(p.phi, loc1.phi, 1e-5);

	EXPECT_NEAR(vl.vx, vel1.vx, 1e-5);
	EXPECT_NEAR(vl.vy, vel1.vy, 1e-5);
	EXPECT_NEAR(vl.omega, vel1.omega, 1e-5);

	EXPECT_NEAR(vg.vx, -vel1.vy, 1e-5);  // 90deg rot
	EXPECT_NEAR(vg.vy, vel1.vx, 1e-5);
	EXPECT_NEAR(vg.omega, vel1.omega, 1e-5);

	// t1:
	rpe.processUpdateNewOdometry(odo2, t1, true, vel1);
	ok = rpe.getCurrentEstimate(p, vl, vg, t1);
	EXPECT_TRUE(ok);
	EXPECT_NEAR(p.x, loc1.x - dt * vel1.vy, 1e-5);
	EXPECT_NEAR(p.y, loc1.y + dt * vel1.vx, 1e-5);
	EXPECT_NEAR(p.phi, loc1.phi + dt * vel1.omega, 1e-5);

	// reset:
	rpe.reset();
	ok = rpe.getCurrentEstimate(p, vl, vg, t1);
	EXPECT_FALSE(ok);
}
