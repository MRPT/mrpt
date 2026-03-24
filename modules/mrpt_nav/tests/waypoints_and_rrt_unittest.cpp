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
#include <mrpt/nav/planners/PlannerRRT_SE2_TPS.h>
#include <mrpt/nav/reactive/TWaypoint.h>

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
