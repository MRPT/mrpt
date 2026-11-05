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
#include <mrpt/math/wrap2pi.h>
#include <mrpt/random.h>

#include <cmath>

using namespace mrpt::kinematics;

namespace
{
/** Gives tests access to the internal control period. */
class DiffDrivenWithPeriod : public CVehicleSimul_DiffDriven
{
 public:
  void setControlPeriod(double h) { m_firmware_control_period = h; }
};

/** Std. dev. of the GT-vs-odometry x error after driving 1 m straight at 1 m/s. */
double straightLineErrorStd(double controlPeriod)
{
  mrpt::random::getRandomGenerator().randomize(1234);
  const int nRuns = 300;
  double sum = 0;
  double sum2 = 0;
  for (int k = 0; k < nRuns; k++)
  {
    DiffDrivenWithPeriod sim;
    sim.setControlPeriod(controlPeriod);
    sim.setOdometryErrors(true, .0, 0.1, .0, .0, .0, .0);
    sim.movementCommand(1.0, .0);
    sim.simulateOneTimeStep(1.0);
    const double e = sim.getCurrentGTPose().x - sim.getCurrentOdometricPose().x;
    sum += e;
    sum2 += e * e;
  }
  const double mean = sum / nRuns;
  return std::sqrt(sum2 / nRuns - mean * mean);
}

/** Forward/lateral displacement of `p` w.r.t. the start, in the start heading frame. */
void forwardLateral(const mrpt::math::TPose2D& p, double heading0, double& fwd, double& lat)
{
  fwd = p.x * std::cos(heading0) + p.y * std::sin(heading0);
  lat = -p.x * std::sin(heading0) + p.y * std::cos(heading0);
}
}  // namespace

// A zero time step must not advance the clock nor move the robot
TEST(CVehicleSimul_DiffDriven, zero_dt_is_a_noop)
{
  CVehicleSimul_DiffDriven sim;
  sim.movementCommand(1.0, 0.5);
  sim.simulateOneTimeStep(0.5);
  const double t0 = sim.getTime();
  const auto p0 = sim.getCurrentGTPose();

  sim.simulateOneTimeStep(.0);
  EXPECT_DOUBLE_EQ(sim.getTime(), t0);
  EXPECT_DOUBLE_EQ(sim.getCurrentGTPose().x, p0.x);
  EXPECT_DOUBLE_EQ(sim.getCurrentGTPose().y, p0.y);
  EXPECT_DOUBLE_EQ(sim.getCurrentGTPose().phi, p0.phi);
}

// Each call must advance exactly dt
TEST(CVehicleSimul_DiffDriven, many_small_steps_advance_exactly_the_requested_time)
{
  for (const int nCalls : {100, 1000})
  {
    CVehicleSimul_DiffDriven sim;
    sim.movementCommand(1.0, .0);
    const double dt = 1.0 / nCalls;
    for (int i = 0; i < nCalls; i++)
    {
      sim.simulateOneTimeStep(dt);
    }
    EXPECT_NEAR(sim.getTime(), 1.0, 1e-9) << "nCalls=" << nCalls;
    EXPECT_NEAR(sim.getCurrentGTPose().x, 1.0, 2e-3) << "nCalls=" << nCalls;
  }
}

// TAU=0 with a pure delay must keep the state finite
TEST(CVehicleSimul_DiffDriven, pure_delay_without_low_pass_stays_finite)
{
  CVehicleSimul_DiffDriven sim;
  sim.setDelayModelParams(.0 /*TAU*/, 0.2 /*DELAY*/);
  sim.movementCommand(1.0, 0.5);

  sim.simulateOneTimeStep(0.1);
  EXPECT_TRUE(std::isfinite(sim.getV()));
  EXPECT_NEAR(sim.getV(), .0, 1e-12);  // still within the delay

  sim.simulateOneTimeStep(0.3);
  EXPECT_NEAR(sim.getV(), 1.0, 1e-12);
  EXPECT_NEAR(sim.getW(), 0.5, 1e-12);
  ASSERT_TRUE(std::isfinite(sim.getCurrentGTPose().x));
  ASSERT_TRUE(std::isfinite(sim.getCurrentGTPose().phi));
  // About 0.2 s of motion at 1 m/s on a gentle arc:
  EXPECT_NEAR(sim.getCurrentGTPose().x, 0.2, 5e-3);
}

// resetStatus() must also reset the low-pass initial speeds
TEST(CVehicleSimul_DiffDriven, resetStatus_cancels_low_pass_residual_motion)
{
  CVehicleSimul_DiffDriven sim;
  sim.setDelayModelParams(1.8 /*TAU*/, .0);
  sim.movementCommand(1.0, 0.2);
  sim.simulateOneTimeStep(5.0);
  sim.movementCommand(1.0, 0.2);  // a new cmd while already moving

  sim.resetStatus();
  sim.simulateOneTimeStep(1.0);
  EXPECT_NEAR(sim.getV(), .0, 1e-12);
  EXPECT_NEAR(sim.getW(), .0, 1e-12);
  EXPECT_NEAR(sim.getCurrentGTPose().x, .0, 1e-12);
  EXPECT_NEAR(sim.getCurrentGTPose().phi, .0, 1e-12);
}

// Odometry error factors apply in the robot frame
TEST(CVehicleSimul_DiffDriven, odometry_bias_is_applied_in_robot_frame)
{
  for (const double heading : {.0, M_PI * 0.25, M_PI * 0.5, -M_PI * 0.75})
  {
    CVehicleSimul_DiffDriven sim;
    sim.setCurrentGTPose(mrpt::math::TPose2D(0, 0, heading));
    sim.setCurrentOdometricPose(mrpt::math::TPose2D(0, 0, heading));
    // Deterministic: a +10% bias in the forward displacement only.
    sim.setOdometryErrors(true, 0.1, .0, .0, .0, .0, .0);
    sim.movementCommand(1.0, .0);
    sim.simulateOneTimeStep(1.0);

    double gtFwd = 0;
    double gtLat = 0;
    double odoFwd = 0;
    double odoLat = 0;
    forwardLateral(sim.getCurrentGTPose(), heading, gtFwd, gtLat);
    forwardLateral(sim.getCurrentOdometricPose(), heading, odoFwd, odoLat);

    EXPECT_NEAR(gtFwd, 1.1 * odoFwd, 1e-6) << "heading=" << heading;
    // A diff-driven robot cannot slip sideways because of a forward bias:
    EXPECT_NEAR(gtLat, .0, 1e-6) << "heading=" << heading;
    EXPECT_NEAR(sim.getCurrentGTPose().phi, heading, 1e-9) << "heading=" << heading;
  }
}

// Odometry noise std is defined per second, independent of the control period
TEST(CVehicleSimul_DiffDriven, odometry_noise_does_not_depend_on_control_period)
{
  const double std1 = straightLineErrorStd(0.5e-3);
  const double std2 = straightLineErrorStd(2.0e-3);
  ASSERT_GT(std1, .0);
  EXPECT_NEAR(std2 / std1, 1.0, 0.25) << "std1=" << std1 << " std2=" << std2;
}

// A pending ramp cmd must keep running after resetTime()
TEST(CVehicleSimul_Holo, resetTime_does_not_freeze_pending_command)
{
  CVehicleSimul_Holo sim;
  sim.sendVelRampCmd(1.0, .0, 0.5, .0);
  sim.simulateOneTimeStep(2.0);
  sim.sendVelRampCmd(.0, .0, 0.5, .0);  // ramp down to a stop
  sim.resetTime();

  const double x0 = sim.getCurrentGTPose().x;
  sim.simulateOneTimeStep(1.0);
  // A linear ramp from 1 m/s to 0 in 0.5 s covers 0.25 m:
  EXPECT_NEAR(sim.getCurrentGTPose().x - x0, 0.25, 5e-3);
  EXPECT_NEAR(sim.getCurrentOdometricVel().vx, .0, 1e-9);
}

// ---------------------------------------------------------------------------
//  Coverage tests (must pass)
// ---------------------------------------------------------------------------

TEST(CVehicleSimul_DiffDriven, negative_dt_is_a_noop)
{
  CVehicleSimul_DiffDriven sim;
  sim.movementCommand(1.0, 0.5);
  sim.simulateOneTimeStep(0.5);
  const double t0 = sim.getTime();
  const auto p0 = sim.getCurrentGTPose();

  sim.simulateOneTimeStep(-0.3);
  EXPECT_DOUBLE_EQ(sim.getTime(), t0);
  EXPECT_DOUBLE_EQ(sim.getCurrentGTPose().x, p0.x);
  EXPECT_DOUBLE_EQ(sim.getCurrentGTPose().phi, p0.phi);
}

TEST(CVehicleSimul_DiffDriven, arc_matches_closed_form_unicycle)
{
  for (const double heading0 : {.0, 2.5, -1.0})
  {
    CVehicleSimul_DiffDriven sim;
    sim.setCurrentGTPose(mrpt::math::TPose2D(1.0, -2.0, heading0));
    sim.setCurrentOdometricPose(mrpt::math::TPose2D(1.0, -2.0, heading0));
    const double v = 0.8;
    const double w = -0.6;
    sim.movementCommand(v, w);
    sim.simulateOneTimeStep(3.0);

    const double t = sim.getTime();
    const double phiEnd = heading0 + w * t;
    const double xExact = 1.0 + v / w * (std::sin(phiEnd) - std::sin(heading0));
    const double yExact = -2.0 - v / w * (std::cos(phiEnd) - std::cos(heading0));

    const auto& p = sim.getCurrentGTPose();
    // First-order integration at 0.5 ms: a few mm of error are expected.
    EXPECT_NEAR(p.x, xExact, 3e-3) << "heading0=" << heading0;
    EXPECT_NEAR(p.y, yExact, 3e-3) << "heading0=" << heading0;
    EXPECT_NEAR(mrpt::math::angDistance(p.phi, mrpt::math::wrapToPi(phiEnd)), .0, 2e-3);

    // Noise-free odometry reproduces the ground truth exactly:
    EXPECT_NEAR(sim.getCurrentOdometricPose().x, p.x, 1e-9);
    EXPECT_NEAR(sim.getCurrentOdometricPose().y, p.y, 1e-9);
    EXPECT_NEAR(sim.getCurrentOdometricPose().phi, p.phi, 1e-9);
  }
}

TEST(CVehicleSimul_DiffDriven, heading_is_wrapped_while_spinning)
{
  CVehicleSimul_DiffDriven sim;
  sim.movementCommand(.0, 2.0);
  sim.simulateOneTimeStep(4.0);  // 8 rad
  const double phi = sim.getCurrentGTPose().phi;
  EXPECT_LE(std::abs(phi), M_PI + 1e-12);
  EXPECT_NEAR(mrpt::math::angDistance(phi, mrpt::math::wrapToPi(8.0)), .0, 5e-3);
  EXPECT_LE(std::abs(sim.getCurrentOdometricPose().phi), M_PI + 1e-12);
}

TEST(CVehicleSimul_DiffDriven, gt_and_odometry_frames_can_differ)
{
  // GT and odometry headings differ by a fixed offset (e.g. after a teleport):
  // the robot must move "forward" in both frames by the same distance.
  CVehicleSimul_DiffDriven sim;
  sim.setCurrentGTPose(mrpt::math::TPose2D(5.0, 1.0, 1.0));
  sim.setCurrentOdometricPose(mrpt::math::TPose2D(.0, .0, -0.5));
  sim.movementCommand(1.0, .0);
  sim.simulateOneTimeStep(2.0);

  const auto& g = sim.getCurrentGTPose();
  const auto& o = sim.getCurrentOdometricPose();
  const double dg = std::hypot(g.x - 5.0, g.y - 1.0);
  const double dodo = std::hypot(o.x, o.y);
  EXPECT_NEAR(dg, dodo, 1e-9);
  EXPECT_NEAR(std::atan2(g.y - 1.0, g.x - 5.0), 1.0, 1e-9);
  EXPECT_NEAR(std::atan2(o.y, o.x), -0.5, 1e-9);

  const auto vg = sim.getCurrentGTVel();
  EXPECT_NEAR(vg.vx, std::cos(1.0), 1e-9);
  EXPECT_NEAR(vg.vy, std::sin(1.0), 1e-9);
  const auto vo = sim.getCurrentOdometricVel();
  EXPECT_NEAR(vo.vx, std::cos(-0.5), 1e-9);
  EXPECT_NEAR(vo.vy, std::sin(-0.5), 1e-9);
  const auto vgl = sim.getCurrentGTVelLocal();
  EXPECT_NEAR(vgl.vx, 1.0, 1e-9);
  EXPECT_NEAR(vgl.vy, .0, 1e-9);
}

TEST(CVehicleSimul_DiffDriven, low_pass_follows_first_order_response)
{
  const double tau = 0.5;
  const double delay = 0.2;
  CVehicleSimul_DiffDriven sim;
  sim.setDelayModelParams(tau, delay);
  sim.movementCommand(2.0, -1.0);

  for (const double t : {0.1, 0.3, 0.7, 1.5})
  {
    sim.simulateOneTimeStep(t - sim.getTime());
    // The controller output lags the clock by up to two control periods:
    const double te = std::max(.0, sim.getTime() - delay);
    const double k = 1.0 - std::exp(-te / tau);
    EXPECT_NEAR(sim.getV(), 2.0 * k, 5e-3) << "t=" << t;
    EXPECT_NEAR(sim.getW(), -1.0 * k, 3e-3) << "t=" << t;
  }
}

TEST(CVehicleSimul_Holo, ramp_is_linear_in_odometry_frame)
{
  CVehicleSimul_Holo sim;
  const double T = 0.4;
  sim.sendVelRampCmd(1.0, M_PI * 0.25, T, .0);

  sim.simulateOneTimeStep(T * 0.5);
  const double c = std::cos(M_PI * 0.25);
  // Half-way through the ramp, half of the target velocity:
  EXPECT_NEAR(sim.getCurrentOdometricVel().vx, 0.5 * c, 5e-3);
  EXPECT_NEAR(sim.getCurrentOdometricVel().vy, 0.5 * c, 5e-3);

  sim.simulateOneTimeStep(1.0);
  // Linear ramp from 0: the distance covered after the ramp is v*(t - T/2).
  const double t = sim.getTime();
  const double dist = std::hypot(sim.getCurrentGTPose().x, sim.getCurrentGTPose().y);
  EXPECT_NEAR(dist, 1.0 * (t - T * 0.5), 3e-3);
  EXPECT_NEAR(std::atan2(sim.getCurrentGTPose().y, sim.getCurrentGTPose().x), M_PI * 0.25, 1e-6);
}

TEST(CVehicleSimul_Holo, dir_local_is_relative_to_current_heading)
{
  const double heading = 2.8;
  const double dirLocal = 0.9;  // heading+dirLocal crosses +pi
  CVehicleSimul_Holo sim;
  sim.setCurrentGTPose(mrpt::math::TPose2D(0, 0, heading));
  sim.setCurrentOdometricPose(mrpt::math::TPose2D(0, 0, heading));
  sim.sendVelCmd(CVehicleVelCmd_Holo(1.0, dirLocal, 0.05, 1.0));
  sim.simulateOneTimeStep(3.0);

  const auto& p = sim.getCurrentGTPose();
  const double moveDir = std::atan2(p.y, p.x);
  EXPECT_NEAR(mrpt::math::angDistance(moveDir, mrpt::math::wrapToPi(heading + dirLocal)), .0, 1e-3);
  // The robot turned the short way round and aligned with the motion:
  EXPECT_NEAR(
      mrpt::math::angDistance(p.phi, mrpt::math::wrapToPi(heading + dirLocal)), .0,
      mrpt::DEG2RAD(1.5));
}
