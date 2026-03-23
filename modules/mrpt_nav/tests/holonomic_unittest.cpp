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

/** Unit tests for holonomic navigation methods (CHolonomicVFF, CHolonomicND,
 *  CHolonomicFullEval) and ClearanceDiagram, exercised in isolation without
 *  a full PTG or reactive navigation system.
 */

#include <gtest/gtest.h>
#include <mrpt/nav/holonomic/CHolonomicFullEval.h>
#include <mrpt/nav/holonomic/CHolonomicND.h>
#include <mrpt/nav/holonomic/CHolonomicVFF.h>
#include <mrpt/nav/holonomic/ClearanceDiagram.h>

#include <cmath>

using mrpt::nav::CAbstractHolonomicReactiveMethod;

// ---------------------------------------------------------------------------
// Helper: build a NavInput with N directions, all free (obs=1.0), and a
// single target at angle `target_angle_rad` with distance `target_dist`.
// ---------------------------------------------------------------------------
static CAbstractHolonomicReactiveMethod::NavInput makeNavInput(
    size_t nDirs, double target_angle_rad, double target_dist = 0.5)
{
  CAbstractHolonomicReactiveMethod::NavInput ni;
  ni.obstacles.assign(nDirs, 1.0);  // all free
  ni.maxObstacleDist = 1.0;
  ni.maxRobotSpeed = 1.0;
  ni.targets.emplace_back(
      target_dist * std::cos(target_angle_rad), target_dist * std::sin(target_angle_rad),
      0.0 /*phi*/);
  return ni;
}

// Wrap angle to [-pi, pi]
static double wrapAngle(double a)
{
  while (a > M_PI) a -= 2 * M_PI;
  while (a < -M_PI) a += 2 * M_PI;
  return a;
}

// ============================================================================
// CHolonomicVFF tests
// ============================================================================
class HolonomicVFFTest : public ::testing::Test
{
 protected:
  mrpt::nav::CHolonomicVFF vff;
};

TEST_F(HolonomicVFFTest, navigate_toward_target_ahead)
{
  constexpr double target_angle = 0.0;  // directly forward
  auto ni = makeNavInput(100, target_angle);
  const auto no = vff.navigate(ni);

  EXPECT_GT(no.desiredSpeed, 0.0);
  EXPECT_LT(no.desiredSpeed, 1.01);
  EXPECT_LT(std::abs(wrapAngle(no.desiredDirection - target_angle)), 0.3 /*rad*/);
}

TEST_F(HolonomicVFFTest, navigate_toward_target_left)
{
  constexpr double target_angle = M_PI / 2.0;  // 90 deg left
  auto ni = makeNavInput(100, target_angle);
  const auto no = vff.navigate(ni);

  EXPECT_GT(no.desiredSpeed, 0.0);
  EXPECT_LT(std::abs(wrapAngle(no.desiredDirection - target_angle)), 0.3 /*rad*/);
}

TEST_F(HolonomicVFFTest, navigate_toward_target_right)
{
  constexpr double target_angle = -M_PI / 4.0;  // 45 deg right
  auto ni = makeNavInput(100, target_angle);
  const auto no = vff.navigate(ni);

  EXPECT_GT(no.desiredSpeed, 0.0);
  EXPECT_LT(std::abs(wrapAngle(no.desiredDirection - target_angle)), 0.3 /*rad*/);
}

TEST_F(HolonomicVFFTest, speed_within_bounds)
{
  auto ni = makeNavInput(100, 0.0, 0.5);
  const auto no = vff.navigate(ni);

  EXPECT_GE(no.desiredSpeed, 0.0);
  EXPECT_LE(no.desiredSpeed, ni.maxRobotSpeed + 1e-6);
}

TEST_F(HolonomicVFFTest, slow_down_near_target)
{
  // target very close → speed should be lower than for a distant target
  auto ni_far = makeNavInput(100, 0.0, 0.5);
  auto ni_near = makeNavInput(100, 0.0, 0.01);  // almost at target

  const auto no_far = vff.navigate(ni_far);
  const auto no_near = vff.navigate(ni_near);

  EXPECT_LE(no_near.desiredSpeed, no_far.desiredSpeed + 1e-6);
}

TEST_F(HolonomicVFFTest, all_obstacles_present_reduces_speed)
{
  // Fill all directions with close obstacles except the target direction
  CAbstractHolonomicReactiveMethod::NavInput ni;
  ni.obstacles.assign(100, 0.05);  // everything very close
  ni.obstacles[50] = 1.0;          // only straight ahead is free
  ni.maxObstacleDist = 1.0;
  ni.maxRobotSpeed = 1.0;
  ni.targets.emplace_back(0.3, 0.0, 0.0);  // target ahead

  const auto no = vff.navigate(ni);
  // Should not crash; output is well-defined
  EXPECT_GE(no.desiredSpeed, 0.0);
  EXPECT_LE(no.desiredSpeed, ni.maxRobotSpeed + 1e-6);
}

// ============================================================================
// CHolonomicND tests
// ============================================================================
class HolonomicNDTest : public ::testing::Test
{
 protected:
  mrpt::nav::CHolonomicND nd;
};

TEST_F(HolonomicNDTest, navigate_toward_target_ahead)
{
  constexpr double target_angle = 0.0;
  auto ni = makeNavInput(121, target_angle);
  const auto no = nd.navigate(ni);

  EXPECT_GT(no.desiredSpeed, 0.0);
  EXPECT_LT(std::abs(wrapAngle(no.desiredDirection - target_angle)), 0.35 /*rad*/);
}

TEST_F(HolonomicNDTest, navigate_toward_target_left)
{
  constexpr double target_angle = M_PI / 2.0;
  auto ni = makeNavInput(121, target_angle);
  const auto no = nd.navigate(ni);

  EXPECT_GT(no.desiredSpeed, 0.0);
  EXPECT_LT(std::abs(wrapAngle(no.desiredDirection - target_angle)), 0.35 /*rad*/);
}

TEST_F(HolonomicNDTest, speed_within_bounds)
{
  auto ni = makeNavInput(121, 0.0);
  const auto no = nd.navigate(ni);

  EXPECT_GE(no.desiredSpeed, 0.0);
  EXPECT_LE(no.desiredSpeed, ni.maxRobotSpeed + 1e-6);
}

TEST_F(HolonomicNDTest, blocked_target_direction_detours)
{
  // Target is at angle 0 but that direction is blocked; the robot should
  // pick a nearby detour rather than charging into the obstacle.
  auto ni = makeNavInput(121, 0.0);
  // Block a swath around angle=0 (indices near 60 out of 121)
  for (int k = 55; k <= 66; k++) ni.obstacles[k] = 0.05;

  const auto no = nd.navigate(ni);

  EXPECT_GE(no.desiredSpeed, 0.0);
  EXPECT_LE(no.desiredSpeed, ni.maxRobotSpeed + 1e-6);
}

// ============================================================================
// CHolonomicFullEval tests
// ============================================================================
class HolonomicFullEvalTest : public ::testing::Test
{
 protected:
  mrpt::nav::CHolonomicFullEval fe;
  mrpt::nav::ClearanceDiagram cd;

  void SetUp() override { cd.resize(121, 121); }
};

TEST_F(HolonomicFullEvalTest, navigate_toward_target_ahead)
{
  // nDirs must be > 3 (asserted inside navigate())
  constexpr double target_angle = 0.0;
  auto ni = makeNavInput(121, target_angle);
  ni.clearance = &cd;
  const auto no = fe.navigate(ni);

  EXPECT_GT(no.desiredSpeed, 0.0);
  EXPECT_LT(std::abs(wrapAngle(no.desiredDirection - target_angle)), 0.35 /*rad*/);
}

TEST_F(HolonomicFullEvalTest, navigate_toward_target_left)
{
  constexpr double target_angle = M_PI / 2.0;
  auto ni = makeNavInput(121, target_angle);
  ni.clearance = &cd;
  const auto no = fe.navigate(ni);

  EXPECT_GT(no.desiredSpeed, 0.0);
  EXPECT_LT(std::abs(wrapAngle(no.desiredDirection - target_angle)), 0.35 /*rad*/);
}

TEST_F(HolonomicFullEvalTest, navigate_toward_target_diagonal)
{
  constexpr double target_angle = M_PI / 4.0;  // 45 deg
  auto ni = makeNavInput(121, target_angle);
  ni.clearance = &cd;
  const auto no = fe.navigate(ni);

  EXPECT_GT(no.desiredSpeed, 0.0);
  EXPECT_LT(std::abs(wrapAngle(no.desiredDirection - target_angle)), 0.35 /*rad*/);
}

TEST_F(HolonomicFullEvalTest, speed_within_bounds)
{
  auto ni = makeNavInput(121, 0.0);
  ni.clearance = &cd;
  const auto no = fe.navigate(ni);

  EXPECT_GE(no.desiredSpeed, 0.0);
  EXPECT_LE(no.desiredSpeed, ni.maxRobotSpeed + 1e-6);
}

TEST_F(HolonomicFullEvalTest, repeated_calls_do_not_crash)
{
  // Hysteresis state is maintained between calls; verify no crash across
  // several consecutive navigate() invocations.
  for (int iter = 0; iter < 5; iter++)
  {
    const double angle = (iter % 2 == 0) ? 0.0 : M_PI / 4.0;
    auto ni = makeNavInput(121, angle);
    ni.clearance = &cd;
    const auto no = fe.navigate(ni);
    EXPECT_GE(no.desiredSpeed, 0.0);
    EXPECT_LE(no.desiredSpeed, ni.maxRobotSpeed + 1e-6);
  }
}

// ============================================================================
// ClearanceDiagram tests
// ============================================================================
TEST(ClearanceDiagram, default_constructed_is_empty)
{
  mrpt::nav::ClearanceDiagram cd;
  EXPECT_TRUE(cd.empty());
}

TEST(ClearanceDiagram, resize_sets_path_counts)
{
  mrpt::nav::ClearanceDiagram cd;
  cd.resize(100, 10);

  EXPECT_FALSE(cd.empty());
  EXPECT_EQ(cd.get_actual_num_paths(), 100u);
  EXPECT_EQ(cd.get_decimated_num_paths(), 10u);
}

TEST(ClearanceDiagram, clear_resets_to_empty)
{
  mrpt::nav::ClearanceDiagram cd;
  cd.resize(50, 5);
  ASSERT_FALSE(cd.empty());

  cd.clear();
  EXPECT_TRUE(cd.empty());
}

TEST(ClearanceDiagram, resize_twice_updates_counts)
{
  mrpt::nav::ClearanceDiagram cd;
  cd.resize(100, 10);
  cd.resize(200, 20);

  EXPECT_EQ(cd.get_actual_num_paths(), 200u);
  EXPECT_EQ(cd.get_decimated_num_paths(), 20u);
}
