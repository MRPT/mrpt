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
#include <mrpt/kinematics/CVehicleSimul_DiffDriven.h>
#include <mrpt/kinematics/CVehicleSimul_Holo.h>
#include <mrpt/random.h>

#include <cmath>

using namespace mrpt::kinematics;

// ---------------------------------------------------------------------------
//  CVehicleSimul_DiffDriven
// ---------------------------------------------------------------------------
TEST(CVehicleSimul_DiffDriven, initial_state_is_zero)
{
  CVehicleSimul_DiffDriven sim;
  EXPECT_EQ(sim.getTime(), .0);
  EXPECT_EQ(sim.getCurrentGTPose().x, .0);
  EXPECT_EQ(sim.getCurrentGTPose().y, .0);
  EXPECT_EQ(sim.getCurrentGTPose().phi, .0);
  EXPECT_EQ(sim.getCurrentOdometricPose().x, .0);
  EXPECT_EQ(sim.getCurrentGTVel().vx, .0);
  EXPECT_EQ(sim.getV(), .0);
  EXPECT_EQ(sim.getW(), .0);
}

TEST(CVehicleSimul_DiffDriven, straight_motion_advances_along_x)
{
  CVehicleSimul_DiffDriven sim;
  sim.movementCommand(1.0 /*m/s*/, .0);
  sim.simulateOneTimeStep(2.0 /*s*/);

  EXPECT_NEAR(sim.getCurrentGTPose().x, 2.0, 0.05);
  EXPECT_NEAR(sim.getCurrentGTPose().y, .0, 1e-6);
  EXPECT_NEAR(sim.getCurrentGTPose().phi, .0, 1e-6);
  EXPECT_NEAR(sim.getTime(), 2.0, 0.01);

  // Without odometry errors, GT and odometry must match:
  EXPECT_NEAR(sim.getCurrentOdometricPose().x, sim.getCurrentGTPose().x, 1e-9);
  EXPECT_NEAR(sim.getCurrentOdometricPose().y, sim.getCurrentGTPose().y, 1e-9);
}

TEST(CVehicleSimul_DiffDriven, pure_rotation_changes_heading_only)
{
  CVehicleSimul_DiffDriven sim;
  sim.movementCommand(.0, 0.5 /*rad/s*/);
  sim.simulateOneTimeStep(2.0);

  EXPECT_NEAR(sim.getCurrentGTPose().phi, 1.0, 0.02);
  EXPECT_NEAR(sim.getCurrentGTPose().x, .0, 1e-6);
  EXPECT_NEAR(sim.getCurrentGTPose().y, .0, 1e-6);
}

TEST(CVehicleSimul_DiffDriven, sendVelCmd_matches_movementCommand)
{
  CVehicleSimul_DiffDriven sim;
  CVehicleVelCmd_DiffDriven cmd;
  cmd.lin_vel = 0.5;
  cmd.ang_vel = .0;
  sim.sendVelCmd(cmd);
  sim.simulateOneTimeStep(1.0);
  EXPECT_NEAR(sim.getCurrentGTPose().x, 0.5, 0.05);
}

TEST(CVehicleSimul_DiffDriven, sendVelCmd_rejects_wrong_kinematic_class)
{
  CVehicleSimul_DiffDriven sim;
  CVehicleVelCmd_Holo wrongCmd;
  EXPECT_ANY_THROW(sim.sendVelCmd(wrongCmd));
}

TEST(CVehicleSimul_DiffDriven, getVelCmdType_returns_diffdriven_cmd)
{
  CVehicleSimul_DiffDriven sim;
  auto c = sim.getVelCmdType();
  ASSERT_TRUE(c);
  EXPECT_EQ(c->getVelCmdLength(), 2U);
  EXPECT_NE(dynamic_cast<CVehicleVelCmd_DiffDriven*>(c.get()), nullptr);
}

TEST(CVehicleSimul_DiffDriven, teleport_and_odometry_override)
{
  CVehicleSimul_DiffDriven sim;
  sim.setCurrentGTPose(mrpt::math::TPose2D(1, 2, 0.3));
  EXPECT_NEAR(sim.getCurrentGTPose().x, 1.0, 1e-12);
  EXPECT_NEAR(sim.getCurrentGTPose().y, 2.0, 1e-12);
  EXPECT_NEAR(sim.getCurrentGTPose().phi, 0.3, 1e-12);

  sim.setCurrentOdometricPose(mrpt::math::TPose2D(4, 5, 0.6));
  EXPECT_NEAR(sim.getCurrentOdometricPose().x, 4.0, 1e-12);
  EXPECT_NEAR(sim.getCurrentOdometricPose().phi, 0.6, 1e-12);
}

TEST(CVehicleSimul_DiffDriven, local_vs_global_velocities)
{
  CVehicleSimul_DiffDriven sim;
  sim.setCurrentGTPose(mrpt::math::TPose2D(0, 0, M_PI * 0.5));
  sim.setCurrentOdometricPose(mrpt::math::TPose2D(0, 0, M_PI * 0.5));
  sim.movementCommand(1.0, .0);
  sim.simulateOneTimeStep(0.5);

  // Moving "forward" while heading +90deg => global velocity is along +y:
  const auto vGlobal = sim.getCurrentGTVel();
  EXPECT_NEAR(vGlobal.vx, .0, 1e-6);
  EXPECT_NEAR(vGlobal.vy, 1.0, 1e-6);

  // ... but in the robot local frame it is purely along +x:
  const auto vLocal = sim.getCurrentGTVelLocal();
  EXPECT_NEAR(vLocal.vx, 1.0, 1e-6);
  EXPECT_NEAR(vLocal.vy, .0, 1e-6);

  const auto vOdoLocal = sim.getCurrentOdometricVelLocal();
  EXPECT_NEAR(vOdoLocal.vx, 1.0, 1e-6);
  EXPECT_NEAR(vOdoLocal.vy, .0, 1e-6);
}

TEST(CVehicleSimul_DiffDriven, resetStatus_and_resetTime)
{
  CVehicleSimul_DiffDriven sim;
  sim.movementCommand(1.0, 0.5);
  sim.simulateOneTimeStep(1.0);
  ASSERT_GT(sim.getTime(), 0.5);

  sim.resetStatus();
  EXPECT_EQ(sim.getCurrentGTPose().x, .0);
  EXPECT_EQ(sim.getCurrentGTPose().phi, .0);
  EXPECT_EQ(sim.getCurrentOdometricPose().x, .0);
  EXPECT_EQ(sim.getCurrentGTVel().vx, .0);
  EXPECT_EQ(sim.getV(), .0);
  EXPECT_EQ(sim.getW(), .0);
  // resetStatus() does not touch the simulation clock:
  EXPECT_GT(sim.getTime(), 0.5);

  sim.resetTime();
  EXPECT_EQ(sim.getTime(), .0);
}

TEST(CVehicleSimul_DiffDriven, setV_setW_are_overwritten_by_control_step)
{
  CVehicleSimul_DiffDriven sim;
  sim.setV(0.5);
  sim.setW(0.1);
  EXPECT_NEAR(sim.getV(), 0.5, 1e-12);
  EXPECT_NEAR(sim.getW(), 0.1, 1e-12);

  // No command was issued, so the low-level controller drives them to zero:
  sim.simulateOneTimeStep(0.1);
  EXPECT_NEAR(sim.getV(), .0, 1e-12);
  EXPECT_NEAR(sim.getW(), .0, 1e-12);
}

TEST(CVehicleSimul_DiffDriven, first_order_delay_model_ramps_up_speed)
{
  CVehicleSimul_DiffDriven sim;
  sim.setDelayModelParams(1.0 /*TAU*/, 0.2 /*DELAY*/);
  sim.movementCommand(1.0, .0);

  // During the pure delay the robot must not have moved yet:
  sim.simulateOneTimeStep(0.1);
  EXPECT_NEAR(sim.getV(), .0, 1e-6);

  // After a while it approaches, but does not reach, the commanded speed:
  sim.simulateOneTimeStep(0.5);
  EXPECT_GT(sim.getV(), .0);
  EXPECT_LT(sim.getV(), 1.0);

  // ... and after several time constants it is nearly there:
  sim.simulateOneTimeStep(5.0);
  EXPECT_NEAR(sim.getV(), 1.0, 0.02);
}

TEST(CVehicleSimul_DiffDriven, odometry_errors_make_gt_and_odometry_diverge)
{
  mrpt::random::getRandomGenerator().randomize(1234);

  CVehicleSimul_DiffDriven sim;
  sim.setOdometryErrors(true, 0.05, 0.02, 0.05, 0.02, 0.05, 0.02);
  sim.movementCommand(1.0, .0);
  sim.simulateOneTimeStep(3.0);

  const double gt_x = sim.getCurrentGTPose().x;
  const double odo_x = sim.getCurrentOdometricPose().x;
  EXPECT_GT(gt_x, .0);
  EXPECT_GT(odo_x, .0);
  EXPECT_GT(std::abs(gt_x - odo_x), 1e-3);

  // Disabling them again makes both evolve identically from now on:
  sim.setOdometryErrors(false);
  const double gt_x0 = sim.getCurrentGTPose().x;
  const double odo_x0 = sim.getCurrentOdometricPose().x;
  sim.simulateOneTimeStep(1.0);
  EXPECT_NEAR(sim.getCurrentGTPose().x - gt_x0, sim.getCurrentOdometricPose().x - odo_x0, 1e-9);
}

// ---------------------------------------------------------------------------
//  CVehicleSimul_Holo
// ---------------------------------------------------------------------------
TEST(CVehicleSimul_Holo, initial_state_is_zero)
{
  CVehicleSimul_Holo sim;
  EXPECT_EQ(sim.getTime(), .0);
  EXPECT_EQ(sim.getCurrentGTPose().x, .0);
  EXPECT_EQ(sim.getCurrentGTVel().vx, .0);
}

TEST(CVehicleSimul_Holo, getVelCmdType_returns_holo_cmd)
{
  CVehicleSimul_Holo sim;
  auto c = sim.getVelCmdType();
  ASSERT_TRUE(c);
  EXPECT_EQ(c->getVelCmdLength(), 4U);
  EXPECT_NE(dynamic_cast<CVehicleVelCmd_Holo*>(c.get()), nullptr);
}

TEST(CVehicleSimul_Holo, vel_ramp_reaches_commanded_velocity)
{
  CVehicleSimul_Holo sim;
  sim.sendVelRampCmd(1.0 /*vel*/, .0 /*dir*/, 0.5 /*ramp_time*/, .0 /*rot_speed*/);

  // Half-way through the ramp, speed is in between:
  sim.simulateOneTimeStep(0.25);
  EXPECT_GT(sim.getCurrentOdometricVel().vx, 0.2);
  EXPECT_LT(sim.getCurrentOdometricVel().vx, 0.8);

  // Once the ramp is over, the target velocity is held:
  sim.simulateOneTimeStep(1.0);
  EXPECT_NEAR(sim.getCurrentOdometricVel().vx, 1.0, 1e-6);
  EXPECT_NEAR(sim.getCurrentOdometricVel().vy, .0, 1e-6);
  EXPECT_GT(sim.getCurrentGTPose().x, 0.5);
}

TEST(CVehicleSimul_Holo, motion_direction_is_honored)
{
  CVehicleSimul_Holo sim;
  sim.sendVelRampCmd(1.0, M_PI * 0.5 /*dir: +90deg*/, 0.2, .0);
  sim.simulateOneTimeStep(1.0);
  EXPECT_NEAR(sim.getCurrentOdometricVel().vx, .0, 1e-6);
  EXPECT_NEAR(sim.getCurrentOdometricVel().vy, 1.0, 1e-6);
  EXPECT_GT(sim.getCurrentGTPose().y, 0.5);
}

TEST(CVehicleSimul_Holo, rotates_until_aligned_with_motion_direction)
{
  CVehicleSimul_Holo sim;
  sim.sendVelRampCmd(0.1, M_PI * 0.5 /*dir*/, 0.2, 1.0 /*rot_speed*/);
  sim.simulateOneTimeStep(5.0);

  // The heading converges to the commanded direction, and rotation stops:
  EXPECT_NEAR(sim.getCurrentOdometricPose().phi, M_PI * 0.5, mrpt::DEG2RAD(2.0));
  EXPECT_NEAR(sim.getCurrentOdometricVel().omega, .0, 1e-9);
}

TEST(CVehicleSimul_Holo, rotation_direction_follows_shortest_angle)
{
  CVehicleSimul_Holo sim;
  // Target heading at -90 deg: the robot must rotate clockwise (omega<0):
  sim.sendVelRampCmd(0.1, -M_PI * 0.5, 0.2, 1.0);
  sim.simulateOneTimeStep(0.3);
  EXPECT_LT(sim.getCurrentOdometricVel().omega, .0);
}

TEST(CVehicleSimul_Holo, sendVelRampCmd_requires_positive_ramp_time)
{
  CVehicleSimul_Holo sim;
  EXPECT_ANY_THROW(sim.sendVelRampCmd(1.0, .0, .0 /*ramp_time*/, .0));
}

TEST(CVehicleSimul_Holo, sendVelCmd_dispatches_to_ramp_command)
{
  CVehicleSimul_Holo sim;
  CVehicleVelCmd_Holo cmd(1.0, .0, 0.3, .0);
  sim.sendVelCmd(cmd);
  sim.simulateOneTimeStep(1.0);
  EXPECT_NEAR(sim.getCurrentOdometricVel().vx, 1.0, 1e-6);
}

TEST(CVehicleSimul_Holo, sendVelCmd_rejects_wrong_kinematic_class)
{
  CVehicleSimul_Holo sim;
  CVehicleVelCmd_DiffDriven wrongCmd;
  EXPECT_ANY_THROW(sim.sendVelCmd(wrongCmd));
}

TEST(CVehicleSimul_Holo, resetStatus_clears_pending_ramp_command)
{
  CVehicleSimul_Holo sim;
  sim.sendVelRampCmd(1.0, .0, 0.2, .0);
  sim.simulateOneTimeStep(1.0);
  ASSERT_GT(sim.getCurrentGTPose().x, 0.1);

  sim.resetStatus();
  EXPECT_EQ(sim.getCurrentGTPose().x, .0);
  EXPECT_EQ(sim.getCurrentOdometricVel().vx, .0);

  // With the command cleared, the robot stays put:
  sim.simulateOneTimeStep(1.0);
  EXPECT_NEAR(sim.getCurrentGTPose().x, .0, 1e-12);
}
