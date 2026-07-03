/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

#include <gtest/gtest.h>
#include <mrpt/poses/CPose2D.h>
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

TEST(CRobot2DPoseEstimator, extrapolateRobotPose_StillAndAckermannArc)
{
  using namespace mrpt::poses;
  using namespace mrpt::math;

  // Still (all velocities zero):
  TPose2D p{1, 2, 0.3};
  TTwist2D vlStill{0, 0, 0};
  TPose2D outStill;
  CRobot2DPoseEstimator::extrapolateRobotPose(p, vlStill, 1.0, outStill);
  EXPECT_NEAR(outStill.x, p.x, 1e-9);
  EXPECT_NEAR(outStill.y, p.y, 1e-9);
  EXPECT_NEAR(outStill.phi, p.phi, 1e-9);

  // Ackermann-like arc (vy==0, non-zero omega):
  TPose2D p0{0, 0, 0};
  TTwist2D vlArc{1.0, 0.0, 0.5};
  TPose2D outArc;
  CRobot2DPoseEstimator::extrapolateRobotPose(p0, vlArc, 1.0, outArc);
  EXPECT_GT(outArc.x, 0.0);
  EXPECT_NEAR(outArc.phi, 0.5, 1e-9);
}

TEST(CRobot2DPoseEstimator, GetLatestRobotPoseBothOverloads)
{
  using namespace mrpt::poses;
  using namespace mrpt::math;

  CRobot2DPoseEstimator rpe;
  TPose2D outNone;
  EXPECT_FALSE(rpe.getLatestRobotPose(outNone));
  EXPECT_NEAR(outNone.x, 0.0, 1e-9);

  const auto t0 = mrpt::Clock::now();
  const auto t1 = mrpt::system::timestampAdd(t0, 1.0);

  const TPose2D loc1{10, 20, 0.1};
  rpe.processUpdateNewPoseLocalization(loc1, t0);

  TPose2D outLocOnly;
  EXPECT_TRUE(rpe.getLatestRobotPose(outLocOnly));
  EXPECT_NEAR(outLocOnly.x, loc1.x, 1e-6);

  CPose2D outLocOnlyCPose;
  EXPECT_TRUE(rpe.getLatestRobotPose(outLocOnlyCPose));
  EXPECT_NEAR(outLocOnlyCPose.x(), loc1.x, 1e-6);

  const TPose2D odo1{0, 0, 0};
  rpe.processUpdateNewOdometry(odo1, t1, false, TTwist2D(0, 0, 0));

  TPose2D outOdoNewer;
  EXPECT_TRUE(rpe.getLatestRobotPose(outOdoNewer));

  // Exercise the dT<=0 warning branch and the hasVelocities=false branch:
  rpe.processUpdateNewOdometry(odo1, t1, false, TTwist2D(0, 0, 0));
}
