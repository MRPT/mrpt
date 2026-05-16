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

/** Unit tests for TWaypoint / TWaypointSequence data structures and
 *  PlannerRRT_SE2_TPS, exercised in isolation.
 */

#include <gtest/gtest.h>
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/core/Clock.h>
#include <mrpt/kinematics/CVehicleVelCmd_DiffDriven.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/nav/planners/PlannerRRT_SE2_TPS.h>
#include <mrpt/nav/reactive/CNavigatorManualSequence.h>
#include <mrpt/nav/reactive/CWaypointsNavigator.h>
#include <mrpt/nav/reactive/TWaypoint.h>

// ============================================================================
// Minimal robot interface mock for navigator tests
// ============================================================================
struct MinimalRobotIF : public mrpt::nav::CRobot2NavInterface
{
  mrpt::math::TPose2D pose{0, 0, 0};
  int changeSpeeds_call_count{0};
  int stop_call_count{0};
  double nav_time{0};

  std::optional<CurrentPoseAndSpeeds> getCurrentPoseAndSpeeds() override
  {
    CurrentPoseAndSpeeds ret;
    ret.pose = pose;
    ret.velGlobal = mrpt::math::TTwist2D(0, 0, 0);
    ret.timestamp = mrpt::Clock::now();
    ret.odometry = pose;
    return ret;
  }

  bool changeSpeeds(const mrpt::kinematics::CVehicleVelCmd&) override
  {
    changeSpeeds_call_count++;
    return true;
  }
  bool stop(bool /*isEmergencyStop*/) override
  {
    stop_call_count++;
    return true;
  }
  mrpt::kinematics::CVehicleVelCmd::Ptr getEmergencyStopCmd() override
  {
    return std::make_shared<mrpt::kinematics::CVehicleVelCmd_DiffDriven>();
  }
  mrpt::kinematics::CVehicleVelCmd::Ptr getStopCmd() override
  {
    return std::make_shared<mrpt::kinematics::CVehicleVelCmd_DiffDriven>();
  }
  bool senseObstacles(mrpt::maps::CSimplePointsMap& obs, mrpt::system::TTimeStamp& ts) override
  {
    obs.clear();
    ts = mrpt::Clock::now();
    return true;
  }
  double getNavigationTime() override { return nav_time; }
  void resetNavigationTimer() override { nav_time = 0; }
};

// Minimal concrete CWaypointsNavigator for testing (always reports reachable)
struct MinimalWaypointsNav : public mrpt::nav::CWaypointsNavigator
{
  using CWaypointsNavigator::CWaypointsNavigator;

  bool impl_waypoint_is_reachable(const mrpt::math::TPoint2D&) const override { return true; }
  void performNavigationStep() override {}
  void loadConfigFile(const mrpt::config::CConfigFileBase&) override {}
  void saveConfigFile(mrpt::config::CConfigFileBase&) const override {}
  void initialize() override {}
};

// ============================================================================
// TWaypoint data structure tests
// ============================================================================
TEST(TWaypoint, default_constructed_is_invalid)
{
  mrpt::nav::TWaypoint wp;
  EXPECT_FALSE(wp.isValid());
}

TEST(TWaypoint, valid_when_fields_set)
{
  mrpt::nav::TWaypoint wp(1.0, 2.0, 0.5);
  EXPECT_TRUE(wp.isValid());
  EXPECT_NEAR(wp.target.x, 1.0, 1e-9);
  EXPECT_NEAR(wp.target.y, 2.0, 1e-9);
  EXPECT_NEAR(wp.allowed_distance, 0.5, 1e-9);
}

TEST(TWaypoint, optional_heading_unset_by_default)
{
  mrpt::nav::TWaypoint wp(0.0, 0.0, 0.5);
  EXPECT_FALSE(wp.target_heading.has_value());
}

TEST(TWaypoint, optional_heading_can_be_set)
{
  mrpt::nav::TWaypoint wp(0.0, 0.0, 0.5, true, 1.57);
  ASSERT_TRUE(wp.target_heading.has_value());
  EXPECT_NEAR(*wp.target_heading, 1.57, 1e-9);
}

TEST(TWaypoint, speed_ratio_defaults_to_one)
{
  mrpt::nav::TWaypoint wp(0.0, 0.0, 0.5);
  EXPECT_NEAR(wp.speed_ratio, 1.0, 1e-9);
}

TEST(TWaypoint, allow_skip_defaults_to_true)
{
  mrpt::nav::TWaypoint wp(0.0, 0.0, 0.5);
  EXPECT_TRUE(wp.allow_skip);
}

TEST(TWaypoint, getAsText_nonempty)
{
  mrpt::nav::TWaypoint wp(1.0, 2.0, 0.5);
  EXPECT_FALSE(wp.getAsText().empty());
}

// ============================================================================
// TWaypointSequence data structure tests
// ============================================================================
TEST(TWaypointSequence, default_constructed_is_empty)
{
  mrpt::nav::TWaypointSequence seq;
  EXPECT_TRUE(seq.waypoints.empty());
}

TEST(TWaypointSequence, can_add_waypoints)
{
  mrpt::nav::TWaypointSequence seq;
  seq.waypoints.emplace_back(1.0, 2.0, 0.5);
  seq.waypoints.emplace_back(3.0, 4.0, 0.5);
  EXPECT_EQ(seq.waypoints.size(), 2u);
}

TEST(TWaypointSequence, clear_empties_sequence)
{
  mrpt::nav::TWaypointSequence seq;
  seq.waypoints.emplace_back(1.0, 2.0, 0.5);
  seq.clear();
  EXPECT_TRUE(seq.waypoints.empty());
}

TEST(TWaypointSequence, getAsText_nonempty)
{
  mrpt::nav::TWaypointSequence seq;
  seq.waypoints.emplace_back(1.0, 2.0, 0.5);
  EXPECT_FALSE(seq.getAsText().empty());
}

// ============================================================================
// TWaypointStatus tests
// ============================================================================
TEST(TWaypointStatus, default_not_reached)
{
  mrpt::nav::TWaypointStatus ws;
  EXPECT_FALSE(ws.reached);
  EXPECT_FALSE(ws.skipped);
}

TEST(TWaypointStatus, assign_from_base_copies_target)
{
  mrpt::nav::TWaypoint wp(5.0, 6.0, 0.3);
  mrpt::nav::TWaypointStatus ws;
  ws = wp;
  EXPECT_NEAR(ws.target.x, 5.0, 1e-9);
  EXPECT_NEAR(ws.target.y, 6.0, 1e-9);
  EXPECT_NEAR(ws.allowed_distance, 0.3, 1e-9);
}

// ============================================================================
// PlannerRRT_SE2_TPS tests
// ============================================================================

// Build a minimal planner using CPTG_Holo_Blend (circular robot, no LUT file).
static void setupRRTPlanner(mrpt::nav::PlannerRRT_SE2_TPS& planner)
{
  const std::string cfg_text =
      "[PTG_CONFIG]\n"
      "MAX_REFERENCE_DISTANCE = 5.0\n"
      "robot_shape_circular_radius = 0.3\n"
      "PTG_COUNT = 1\n"
      "PTG0_Type = CPTG_Holo_Blend\n"
      "PTG0_refDistance = 5.0\n"
      "PTG0_num_paths = 31\n"
      "PTG0_v_max_mps = 1.0\n"
      "PTG0_w_max_dps = 90\n"
      "PTG0_T_ramp_max = 0.9\n"
      "PTG0_V_MAX = 1.0\n"
      "PTG0_W_MAX = 1.5708\n";

  mrpt::config::CConfigFileMemory cfg(cfg_text);
  planner.loadConfig(cfg);

  // Quick termination: 1 s is plenty for an open-space nearby goal.
  planner.end_criteria.maxComputationTime = 1.0;
  planner.end_criteria.acceptedDistToTarget = 0.4;
  planner.params.ptg_verbose = false;
  planner.params.ptg_cache_files_directory = "/tmp";

  planner.initialize();
}

TEST(PlannerRRTSE2TPS, initialize_does_not_throw)
{
  mrpt::nav::PlannerRRT_SE2_TPS planner;
  EXPECT_NO_THROW(setupRRTPlanner(planner));
}

TEST(PlannerRRTSE2TPS, solves_very_close_goal)
{
  mrpt::nav::PlannerRRT_SE2_TPS planner;
  setupRRTPlanner(planner);

  mrpt::nav::PlannerRRT_SE2_TPS::TPlannerInput pi;
  pi.start_pose = mrpt::math::TPose2D(0, 0, 0);
  // Goal just within the acceptance radius (0.4 m) — should succeed immediately
  pi.goal_pose = mrpt::math::TPose2D(0.3, 0, 0);
  pi.world_bbox_min = mrpt::math::TPose2D(-5, -5, -M_PI);
  pi.world_bbox_max = mrpt::math::TPose2D(5, 5, M_PI);

  mrpt::nav::PlannerRRT_SE2_TPS::TPlannerResult result;
  EXPECT_NO_THROW(planner.solve(pi, result));
  EXPECT_TRUE(result.success);
}

TEST(PlannerRRTSE2TPS, solves_nearby_goal_open_space)
{
  mrpt::nav::PlannerRRT_SE2_TPS planner;
  setupRRTPlanner(planner);

  mrpt::nav::PlannerRRT_SE2_TPS::TPlannerInput pi;
  pi.start_pose = mrpt::math::TPose2D(0, 0, 0);
  pi.goal_pose = mrpt::math::TPose2D(1.5, 0, 0);  // 1.5 m ahead, no obstacles
  pi.world_bbox_min = mrpt::math::TPose2D(-5, -5, -M_PI);
  pi.world_bbox_max = mrpt::math::TPose2D(5, 5, M_PI);

  mrpt::nav::PlannerRRT_SE2_TPS::TPlannerResult result;
  EXPECT_NO_THROW(planner.solve(pi, result));
  EXPECT_TRUE(result.success);
  EXPECT_GT(result.move_tree.getAllNodes().size(), 0u);
}

TEST(PlannerRRTSE2TPS, result_tree_not_empty_after_solve)
{
  mrpt::nav::PlannerRRT_SE2_TPS planner;
  setupRRTPlanner(planner);

  mrpt::nav::PlannerRRT_SE2_TPS::TPlannerInput pi;
  pi.start_pose = mrpt::math::TPose2D(0, 0, 0);
  pi.goal_pose = mrpt::math::TPose2D(2.0, 1.0, 0);
  pi.world_bbox_min = mrpt::math::TPose2D(-5, -5, -M_PI);
  pi.world_bbox_max = mrpt::math::TPose2D(5, 5, M_PI);

  mrpt::nav::PlannerRRT_SE2_TPS::TPlannerResult result;
  planner.solve(pi, result);

  // Tree must have at least the root node
  EXPECT_GT(result.move_tree.getAllNodes().size(), 0u);
}

// ============================================================================
// CWaypointsNavigator state machine tests
// ============================================================================

TEST(CWaypointsNavigator, navigateWaypoints_sets_sequence)
{
  MinimalRobotIF robot;
  MinimalWaypointsNav nav(robot);
  nav.setMinLoggingLevel(mrpt::system::LVL_ERROR);

  mrpt::nav::TWaypointSequence seq;
  seq.waypoints.emplace_back(1.0, 0.0, 0.5);
  seq.waypoints.emplace_back(2.0, 0.0, 0.5);
  nav.navigateWaypoints(seq);

  const auto status = nav.getWaypointNavStatus();
  EXPECT_EQ(status.waypoints.size(), 2u);
  EXPECT_NEAR(status.waypoints[0].target.x, 1.0, 1e-9);
  EXPECT_NEAR(status.waypoints[1].target.x, 2.0, 1e-9);
}

TEST(CWaypointsNavigator, cancel_clears_active_navigation)
{
  MinimalRobotIF robot;
  MinimalWaypointsNav nav(robot);
  nav.setMinLoggingLevel(mrpt::system::LVL_ERROR);

  mrpt::nav::TWaypointSequence seq;
  seq.waypoints.emplace_back(5.0, 0.0, 0.5);
  nav.navigateWaypoints(seq);
  nav.cancel();

  // After cancel the navigator should be IDLE (not NAVIGATING):
  EXPECT_NE(nav.getCurrentState(), mrpt::nav::CAbstractNavigator::TState::NAVIGATING);
}

TEST(CWaypointsNavigator, getWaypointsAccessGuard_provides_mutable_access)
{
  MinimalRobotIF robot;
  MinimalWaypointsNav nav(robot);
  nav.setMinLoggingLevel(mrpt::system::LVL_ERROR);

  mrpt::nav::TWaypointSequence seq;
  seq.waypoints.emplace_back(1.0, 0.0, 0.5);
  nav.navigateWaypoints(seq);

  {
    auto guard = nav.getWaypointsAccessGuard();
    ASSERT_FALSE(guard.waypoints().waypoints.empty());
    guard.waypoints().waypoints[0].allow_skip = true;
  }

  const auto status = nav.getWaypointNavStatus();
  EXPECT_TRUE(status.waypoints[0].allow_skip);
}

TEST(CWaypointsNavigator, navigationStep_does_not_crash_without_prior_navigate)
{
  MinimalRobotIF robot;
  MinimalWaypointsNav nav(robot);
  nav.setMinLoggingLevel(mrpt::system::LVL_ERROR);

  // Should not throw; with no navigate command the state is IDLE.
  EXPECT_NO_THROW(nav.navigationStep());
  EXPECT_EQ(nav.getCurrentState(), mrpt::nav::CAbstractNavigator::TState::IDLE);
}

// ============================================================================
// CNavigatorManualSequence tests
// ============================================================================

TEST(CNavigatorManualSequence, executes_programmed_command_at_correct_time)
{
  MinimalRobotIF robot;
  mrpt::nav::CNavigatorManualSequence nav(robot);
  nav.setMinLoggingLevel(mrpt::system::LVL_ERROR);

  // Program a single DiffDriven command at t=0s:
  mrpt::nav::CNavigatorManualSequence::TVelCmd krc;
  auto cmd = std::make_shared<mrpt::kinematics::CVehicleVelCmd_DiffDriven>();
  cmd->setVelCmdElement(0, 0.5);  // v
  cmd->setVelCmdElement(1, 0.0);  // w
  krc.cmd_vel = cmd;
  nav.programmed_orders[0.0] = krc;

  nav.initialize();

  // t=0: command should fire
  robot.nav_time = 0.0;
  EXPECT_EQ(robot.changeSpeeds_call_count, 0);
  nav.navigationStep();
  EXPECT_EQ(robot.changeSpeeds_call_count, 1);

  // Order should have been consumed:
  EXPECT_TRUE(nav.programmed_orders.empty());
}

TEST(CNavigatorManualSequence, does_not_fire_command_before_scheduled_time)
{
  MinimalRobotIF robot;
  mrpt::nav::CNavigatorManualSequence nav(robot);
  nav.setMinLoggingLevel(mrpt::system::LVL_ERROR);

  mrpt::nav::CNavigatorManualSequence::TVelCmd krc;
  auto cmd = std::make_shared<mrpt::kinematics::CVehicleVelCmd_DiffDriven>();
  cmd->setVelCmdElement(0, 0.3);
  cmd->setVelCmdElement(1, 0.0);
  krc.cmd_vel = cmd;
  nav.programmed_orders[1.0] = krc;  // scheduled at t=1s

  nav.initialize();

  // t=0.5: too early
  robot.nav_time = 0.5;
  nav.navigationStep();
  EXPECT_EQ(robot.changeSpeeds_call_count, 0);

  // t=1.0: fires now
  robot.nav_time = 1.0;
  nav.navigationStep();
  EXPECT_EQ(robot.changeSpeeds_call_count, 1);
}

TEST(CNavigatorManualSequence, empty_orders_step_is_no_op)
{
  MinimalRobotIF robot;
  mrpt::nav::CNavigatorManualSequence nav(robot);
  nav.setMinLoggingLevel(mrpt::system::LVL_ERROR);

  // No programmed orders — navigationStep should be safe
  robot.nav_time = 0.0;
  EXPECT_NO_THROW(nav.navigationStep());
  EXPECT_EQ(robot.changeSpeeds_call_count, 0);
}
