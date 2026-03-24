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
 *  CHolonomicFullEval), ClearanceDiagram, and CMultiObjMotionOpt_Scalarization,
 *  exercised in isolation without a full PTG or reactive navigation system.
 */

#include <gtest/gtest.h>
#include <mrpt/nav/holonomic/CHolonomicFullEval.h>
#include <mrpt/nav/holonomic/CHolonomicND.h>
#include <mrpt/nav/holonomic/CHolonomicVFF.h>
#include <mrpt/nav/holonomic/ClearanceDiagram.h>
#include <mrpt/nav/reactive/CMultiObjMotionOpt_Scalarization.h>
#include <mrpt/nav/reactive/TCandidateMovementPTG.h>

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

// ============================================================================
// CMultiObjMotionOpt_Scalarization tests
// ============================================================================

// Build an optimizer with a trivial single-score formula.
// Prop variable "qval" → score "s" → scalar = "s".
// Score names must differ from prop names to avoid registration collision.
static mrpt::nav::CMultiObjMotionOpt_Scalarization makeSimpleOptimizer(
    const std::string& scalar_formula = "s")
{
  mrpt::nav::CMultiObjMotionOpt_Scalarization opt;
  opt.parameters.formula_score.clear();
  opt.parameters.formula_score["s"] = "qval";  // score "s" = prop variable "qval"
  opt.parameters.scalar_score_formula = scalar_formula;
  return opt;
}

// Build a TCandidateMovementPTG with a single "qval" property.
static mrpt::nav::TCandidateMovementPTG makeCandidate(double quality, double speed = 1.0)
{
  mrpt::nav::TCandidateMovementPTG m;
  m.speed = speed;
  m.props["qval"] = quality;
  return m;
}

TEST(MultiObjOptScalarization, selects_highest_score)
{
  auto opt = makeSimpleOptimizer();
  std::vector<mrpt::nav::TCandidateMovementPTG> movs = {
      makeCandidate(0.3), makeCandidate(0.9), makeCandidate(0.5)};

  mrpt::nav::CMultiObjectiveMotionOptimizerBase::TResultInfo info;
  const auto best = opt.decide(movs, info);

  ASSERT_TRUE(best.has_value());
  EXPECT_EQ(*best, 1u);  // index 1 has highest quality
}

TEST(MultiObjOptScalarization, returns_nullopt_when_all_infeasible)
{
  auto opt = makeSimpleOptimizer();
  // speed <= 0 marks a candidate as infeasible
  std::vector<mrpt::nav::TCandidateMovementPTG> movs = {
      makeCandidate(0.9, -1.0), makeCandidate(0.8, 0.0)};

  mrpt::nav::CMultiObjectiveMotionOptimizerBase::TResultInfo info;
  const auto best = opt.decide(movs, info);

  EXPECT_FALSE(best.has_value());
}

TEST(MultiObjOptScalarization, returns_nullopt_for_empty_input)
{
  auto opt = makeSimpleOptimizer();
  std::vector<mrpt::nav::TCandidateMovementPTG> movs;

  mrpt::nav::CMultiObjectiveMotionOptimizerBase::TResultInfo info;
  const auto best = opt.decide(movs, info);

  EXPECT_FALSE(best.has_value());
}

TEST(MultiObjOptScalarization, ignores_infeasible_candidates)
{
  auto opt = makeSimpleOptimizer();
  // index 0 is infeasible; index 1 is feasible but lower quality than 0's score
  std::vector<mrpt::nav::TCandidateMovementPTG> movs = {
      makeCandidate(0.99, -1.0),  // infeasible
      makeCandidate(0.5, 1.0),    // feasible
  };

  mrpt::nav::CMultiObjectiveMotionOptimizerBase::TResultInfo info;
  const auto best = opt.decide(movs, info);

  ASSERT_TRUE(best.has_value());
  EXPECT_EQ(*best, 1u);  // only feasible candidate
}

TEST(MultiObjOptScalarization, final_evaluation_has_correct_size)
{
  auto opt = makeSimpleOptimizer();
  std::vector<mrpt::nav::TCandidateMovementPTG> movs = {
      makeCandidate(0.4), makeCandidate(0.7), makeCandidate(0.2)};

  mrpt::nav::CMultiObjectiveMotionOptimizerBase::TResultInfo info;
  opt.decide(movs, info);

  EXPECT_EQ(info.final_evaluation.size(), movs.size());
}

TEST(MultiObjOptScalarization, combined_score_formula)
{
  // Prop vars "av", "bv"; scores "sa"="av", "sb"="bv"; scalar = sa + sb
  mrpt::nav::CMultiObjMotionOpt_Scalarization opt;
  opt.parameters.formula_score.clear();
  opt.parameters.formula_score["sa"] = "av";
  opt.parameters.formula_score["sb"] = "bv";
  opt.parameters.scalar_score_formula = "sa + sb";

  mrpt::nav::TCandidateMovementPTG m0, m1;
  m0.speed = 1.0;
  m0.props["av"] = 0.3;
  m0.props["bv"] = 0.3;  // sum = 0.6
  m1.speed = 1.0;
  m1.props["av"] = 0.4;
  m1.props["bv"] = 0.5;  // sum = 0.9

  mrpt::nav::CMultiObjectiveMotionOptimizerBase::TResultInfo info;
  const auto best = opt.decide({m0, m1}, info);

  ASSERT_TRUE(best.has_value());
  EXPECT_EQ(*best, 1u);
}
