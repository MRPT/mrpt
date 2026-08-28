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

/** Path-planner coverage beyond the "does it find a solution" tests: the
 *  RRT move-tree 3D rendering, planner configuration variants, and a full
 *  round-trip of the (heavily populated) reactive-navigation log record.
 */

#include <gtest/gtest.h>
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/kinematics/CVehicleVelCmd_DiffDriven.h>
#include <mrpt/nav/holonomic/CHolonomicFullEval.h>
#include <mrpt/nav/planners/PlannerRRT_SE2_TPS.h>
#include <mrpt/nav/planners/PlannerSimple2D.h>
#include <mrpt/nav/reactive/CLogFileRecord.h>
#include <mrpt/nav/tpspace/CPTG_DiffDrive_C.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/viz/Scene.h>

using namespace mrpt::nav;

namespace
{
const char* const kRRTCfgCircular =
    "[PTG_CONFIG]\n"
    "robot_shape_circular_radius = 0.3\n"
    "PTG_COUNT = 1\n"
    "PTG0_Type = CPTG_Holo_Blend\n"
    "PTG0_refDistance = 4.0\n"
    "PTG0_num_paths = 21\n"
    "PTG0_v_max_mps = 1.0\n"
    "PTG0_w_max_dps = 90\n"
    "PTG0_T_ramp_max = 0.9\n";

/** Same, but with an explicit polygonal robot shape (MATLAB matrix syntax). */
const char* const kRRTCfgPolygonal =
    "[PTG_CONFIG]\n"
    "robot_shape = [-0.2 0.2 0.2 -0.2; -0.1 -0.1 0.1 0.1]\n"
    "PTG_COUNT = 1\n"
    "PTG0_Type = CPTG_DiffDrive_C\n"
    "PTG0_refDistance = 3.0\n"
    "PTG0_resolution = 0.25\n"
    "PTG0_num_paths = 21\n"
    "PTG0_v_max_mps = 1.0\n"
    "PTG0_w_max_dps = 90\n"
    "PTG0_K = 1.0\n";

void setup_planner(PlannerRRT_SE2_TPS& planner, const char* cfgText)
{
  mrpt::config::CConfigFileMemory cfg(std::string{cfgText});
  planner.loadConfig(cfg);
  planner.end_criteria.maxComputationTime = 1.0;
  planner.end_criteria.acceptedDistToTarget = 0.4;
  planner.params.ptg_verbose = false;
  planner.params.ptg_cache_files_directory =
      mrpt::system::extractFileDirectory(mrpt::system::getTempFileName());
  planner.initialize();
}

PlannerRRT_SE2_TPS::TPlannerInput make_planner_input()
{
  PlannerRRT_SE2_TPS::TPlannerInput pi;
  pi.start_pose = mrpt::math::TPose2D(0, 0, 0);
  pi.goal_pose = mrpt::math::TPose2D(2.0, 0.5, 0);
  pi.world_bbox_min = mrpt::math::TPose2D(-5, -5, -M_PI);
  pi.world_bbox_max = mrpt::math::TPose2D(5, 5, M_PI);
  return pi;
}
}  // namespace

// ---------------------------------------------------------------------------
//  PlannerRRT_SE2_TPS
// ---------------------------------------------------------------------------
TEST(PlannerRRTSE2TPS, loads_a_polygonal_robot_shape)
{
  PlannerRRT_SE2_TPS planner;
  ASSERT_NO_THROW(setup_planner(planner, kRRTCfgPolygonal));
  EXPECT_EQ(planner.params.robot_shape.size(), 4U);
  EXPECT_EQ(planner.getPTGs().size(), 1U);
}

TEST(PlannerRRTSE2TPS, rejects_a_malformed_robot_shape)
{
  PlannerRRT_SE2_TPS planner;
  mrpt::config::CConfigFileMemory cfg(
      std::string{"[PTG_CONFIG]\n"
                  "robot_shape = not a matlab matrix\n"
                  "PTG_COUNT = 0\n"});
  EXPECT_ANY_THROW(planner.loadConfig(cfg));
}

TEST(PlannerRRTSE2TPS, solve_before_initialize_is_rejected)
{
  PlannerRRT_SE2_TPS planner;
  auto pi = make_planner_input();
  PlannerRRT_SE2_TPS::TPlannerResult result;
  EXPECT_ANY_THROW(planner.solve(pi, result));
}

TEST(PlannerRRTSE2TPS, obstacles_are_taken_into_account)
{
  PlannerRRT_SE2_TPS planner;
  setup_planner(planner, kRRTCfgCircular);

  auto pi = make_planner_input();
  // A wall of obstacles right between start and goal:
  for (double y = -2.0; y <= 2.0; y += 0.05)
  {
    pi.obstacles_points.insertPoint(1.0, y, .0);
  }

  PlannerRRT_SE2_TPS::TPlannerResult result;
  EXPECT_NO_THROW(planner.solve(pi, result));
  // Whatever the outcome, the tree must have been grown from the root:
  EXPECT_GT(result.move_tree.getAllNodes().size(), 0U);
  EXPECT_GT(result.computation_time, .0);
}

TEST(PlannerRRTSE2TPS, renders_the_move_tree_into_a_3d_scene)
{
  PlannerRRT_SE2_TPS planner;
  setup_planner(planner, kRRTCfgCircular);

  auto pi = make_planner_input();
  pi.obstacles_points.insertPoint(1.0, 1.0, .0);
  pi.obstacles_points.insertPoint(1.0, -1.0, .0);

  PlannerRRT_SE2_TPS::TPlannerResult result;
  planner.solve(pi, result);

  mrpt::viz::Scene scene;
  PlannerRRT_SE2_TPS::TRenderPlannedPathOptions opts;
  opts.highlight_path_to_node_id = result.best_goal_node_id;
  opts.highlight_last_added_edge = true;
  opts.ground_xy_grid_frequency = 1.0;
  opts.log_msg = "unit test";

  const mrpt::poses::CPose2D xRand(1, 1, 0), xNearest(0.5, 0.5, 0), newState(1.5, 0.5, 0);
  opts.x_rand_pose = &xRand;
  opts.x_nearest_pose = &xNearest;
  opts.new_state = &newState;
  opts.local_obs_from_nearest_pose = &pi.obstacles_points;

  planner.renderMoveTree(scene, pi, result, opts);
  EXPECT_GT(scene.getViewport()->size(), 0U);
}

TEST(PlannerRRTSE2TPS, rendering_options_can_be_switched_off)
{
  PlannerRRT_SE2_TPS planner;
  setup_planner(planner, kRRTCfgPolygonal);

  auto pi = make_planner_input();
  PlannerRRT_SE2_TPS::TPlannerResult result;
  planner.solve(pi, result);

  mrpt::viz::Scene scene;
  PlannerRRT_SE2_TPS::TRenderPlannedPathOptions opts;
  opts.draw_obstacles = false;
  opts.ground_xy_grid_frequency = 0;  // disabled
  opts.draw_shape_decimation = 3;
  opts.xyzcorners_scale = 0.5;

  EXPECT_NO_THROW(planner.renderMoveTree(scene, pi, result, opts));
  EXPECT_GT(scene.getViewport()->size(), 0U);
}

TEST(PlannerRRTSE2TPS, an_unsolved_tree_still_renders)
{
  PlannerRRT_SE2_TPS planner;
  setup_planner(planner, kRRTCfgCircular);

  auto pi = make_planner_input();
  PlannerRRT_SE2_TPS::TPlannerResult emptyResult;  // never solved

  mrpt::viz::Scene scene;
  PlannerRRT_SE2_TPS::TRenderPlannedPathOptions opts;
  EXPECT_NO_THROW(planner.renderMoveTree(scene, pi, emptyResult, opts));
}

TEST(PlannerRRTSE2TPS, profiler_is_accessible)
{
  PlannerRRT_SE2_TPS planner;
  EXPECT_NO_THROW(planner.getProfiler().clear(true));
}

// ---------------------------------------------------------------------------
//  PlannerSimple2D
// ---------------------------------------------------------------------------
TEST(PlannerSimple2D, unreachable_goal_yields_an_empty_path)
{
  mrpt::maps::COccupancyGridMap2D grid;
  grid.setSize(-5.0f, 5.0f, -5.0f, 5.0f, 0.10f);
  grid.fill(0.6f);

  // A wall fully splitting the map in two:
  for (float y = -5.0f; y <= 5.0f; y += 0.05f)
  {
    grid.setPos(0.0f, y, 0.0f);
  }

  PlannerSimple2D planner;
  planner.robotRadius = 0.15f;

  std::deque<mrpt::math::TPoint2D> path;
  bool notFound = false;
  planner.computePath(
      grid, mrpt::poses::CPose2D(-2, 0, 0), mrpt::poses::CPose2D(2, 0, 0), path, notFound, 20.0f);

  EXPECT_TRUE(notFound);
  EXPECT_TRUE(path.empty());
}

TEST(PlannerSimple2D, path_is_found_in_an_open_map)
{
  mrpt::maps::COccupancyGridMap2D grid;
  grid.setSize(-5.0f, 5.0f, -5.0f, 5.0f, 0.10f);
  grid.fill(0.6f);

  PlannerSimple2D planner;
  planner.robotRadius = 0.15f;

  std::deque<mrpt::math::TPoint2D> path;
  bool notFound = true;
  planner.computePath(
      grid, mrpt::poses::CPose2D(-2, 0, 0), mrpt::poses::CPose2D(2, 0, 0), path, notFound, -1.0f);

  EXPECT_FALSE(notFound);
  EXPECT_GT(path.size(), 1U);
  EXPECT_NEAR(path.back().x, 2.0, 0.5);
  EXPECT_NEAR(path.back().y, 0.0, 0.5);
}

// ---------------------------------------------------------------------------
//  CLogFileRecord
// ---------------------------------------------------------------------------
TEST(CLogFileRecord, fully_populated_record_survives_a_roundtrip)
{
  // A PTG to embed in the record (as the navigator does for the first entry
  // of every log file):
  mrpt::config::CConfigFileMemory ptgCfg;
  ptgCfg.write("PTG", "num_paths", 21);
  ptgCfg.write("PTG", "refDistance", 2.0);
  ptgCfg.write("PTG", "resolution", 0.25);
  ptgCfg.write("PTG", "v_max_mps", 1.0);
  ptgCfg.write("PTG", "w_max_dps", 60.0);
  ptgCfg.write("PTG", "K", 1.0);
  ptgCfg.write("PTG", "shape_x0", -0.2);
  ptgCfg.write("PTG", "shape_y0", 0.2);
  ptgCfg.write("PTG", "shape_x1", 0.2);
  ptgCfg.write("PTG", "shape_y1", 0.2);
  ptgCfg.write("PTG", "shape_x2", 0.2);
  ptgCfg.write("PTG", "shape_y2", -0.2);
  auto ptg = CParameterizedTrajectoryGenerator::CreatePTG("CPTG_DiffDrive_C", ptgCfg, "PTG", "");

  CLogFileRecord rec;
  rec.nPTGs = 1;
  rec.infoPerPTG.resize(1);
  {
    auto& ipp = rec.infoPerPTG[0];
    ipp.PTG_desc = "a PTG";
    ipp.TP_Obstacles.resize(4);
    ipp.TP_Obstacles.fill(1.5f);
    ipp.TP_Targets.emplace_back(1.0, 2.0, 0.3);
    ipp.TP_Robot = mrpt::math::TPoint2D(0.1, 0.2);
    ipp.timeForTPObsTransformation = 0.01;
    ipp.timeForHolonomicMethod = 0.02;
    ipp.desiredDirection = 0.3;
    ipp.desiredSpeed = 0.4;
    ipp.evaluation = 0.5;
    ipp.evalFactors["clearance"] = 0.6;
    ipp.HLFR = std::make_shared<CLogFileRecord_FullEval>();
    ipp.ptg = ptg;
    ipp.clearance.resize(21, 4);
    ipp.clearance.get_path_clearance_decimated(0)[0.5] = 0.7;
    ipp.dynState.curVelLocal = mrpt::math::TTwist2D(0.5, 0, 0);
    ipp.lastDynState.relTarget = mrpt::math::TPose2D(1, 1, 0);
  }
  rec.nSelectedPTG = 0;
  rec.values["executionTime"] = 0.05;
  rec.timestamps["tim_start_iteration"] = mrpt::Clock::now();
  rec.additional_debug_msgs["msg"] = "hello";
  rec.WS_Obstacles.insertPoint(1, 2, 0);
  rec.WS_Obstacles_original.insertPoint(1, 2, 0);
  rec.robotPoseLocalization = mrpt::math::TPose2D(1, 2, 0.3);
  rec.robotPoseOdometry = mrpt::math::TPose2D(1.1, 2.1, 0.31);
  rec.relPoseSense = mrpt::math::TPose2D(0.01, 0, 0);
  rec.relPoseVelCmd = mrpt::math::TPose2D(0.02, 0, 0);
  rec.WS_targets_relative.emplace_back(3.0, 4.0, 0.0);
  {
    auto c = std::make_shared<mrpt::kinematics::CVehicleVelCmd_DiffDriven>();
    c->lin_vel = 0.5;
    c->ang_vel = 0.1;
    rec.cmd_vel = c;
    rec.cmd_vel_original = c;
  }
  rec.cur_vel = mrpt::math::TTwist2D(0.5, 0, 0.1);
  rec.cur_vel_local = mrpt::math::TTwist2D(0.5, 0, 0.1);
  rec.robotShape_x.resize(3);
  rec.robotShape_y.resize(3);
  rec.robotShape_radius = 0.35;
  rec.ptg_index_NOP = -1;  // not a "NOP cmdvel" step: cmd_vel is stored
  rec.rel_cur_pose_wrt_last_vel_cmd_NOP = mrpt::math::TPose2D(0.1, 0, 0);
  rec.rel_pose_PTG_origin_wrt_sense_NOP = mrpt::math::TPose2D(0.2, 0, 0);
  rec.visuals.push_back(mrpt::viz::CSetOfObjects::Create());

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << rec;
  buf.Seek(0);

  CLogFileRecord rec2;
  arch >> rec2;

  EXPECT_EQ(rec2.nPTGs, rec.nPTGs);
  ASSERT_EQ(rec2.infoPerPTG.size(), 1U);
  EXPECT_EQ(rec2.infoPerPTG[0].PTG_desc, "a PTG");
  EXPECT_EQ(rec2.infoPerPTG[0].TP_Obstacles.size(), 4);
  ASSERT_EQ(rec2.infoPerPTG[0].TP_Targets.size(), 1U);
  EXPECT_NEAR(rec2.infoPerPTG[0].TP_Targets[0].x, 1.0, 1e-6);
  EXPECT_NEAR(rec2.infoPerPTG[0].desiredDirection, 0.3, 1e-9);
  EXPECT_NEAR(rec2.infoPerPTG[0].evalFactors.at("clearance"), 0.6, 1e-9);
  EXPECT_TRUE(rec2.infoPerPTG[0].HLFR);
  EXPECT_TRUE(rec2.infoPerPTG[0].ptg);
  EXPECT_EQ(rec2.infoPerPTG[0].clearance.get_actual_num_paths(), 21U);
  EXPECT_EQ(rec2.nSelectedPTG, 0);
  EXPECT_NEAR(rec2.values.at("executionTime"), 0.05, 1e-9);
  EXPECT_EQ(rec2.additional_debug_msgs.at("msg"), "hello");
  EXPECT_EQ(rec2.WS_Obstacles.size(), 1U);
  EXPECT_NEAR(rec2.robotPoseLocalization.x, 1.0, 1e-9);
  ASSERT_EQ(rec2.WS_targets_relative.size(), 1U);
  EXPECT_NEAR(rec2.WS_targets_relative[0].x, 3.0, 1e-9);
  ASSERT_TRUE(rec2.cmd_vel);
  EXPECT_NEAR(rec2.cmd_vel->getVelCmdElement(0), 0.5, 1e-9);
  EXPECT_NEAR(rec2.robotShape_radius, 0.35, 1e-9);
  EXPECT_EQ(rec2.ptg_index_NOP, -1);
  EXPECT_EQ(rec2.visuals.size(), 1U);
}

TEST(CLogFileRecord, nop_cmdvel_records_store_the_ptg_continuation_fields)
{
  CLogFileRecord rec;
  rec.nPTGs = 0;
  // In "NOP cmdvel" steps no new command is sent; the PTG-continuation
  // bookkeeping is stored instead.
  rec.ptg_index_NOP = 1;
  rec.ptg_last_k_NOP = 7;
  rec.rel_cur_pose_wrt_last_vel_cmd_NOP = mrpt::math::TPose2D(0.1, 0.2, 0.3);
  rec.rel_pose_PTG_origin_wrt_sense_NOP = mrpt::math::TPose2D(0.4, 0.5, 0.6);

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << rec;
  buf.Seek(0);

  CLogFileRecord rec2;
  arch >> rec2;

  EXPECT_EQ(rec2.ptg_index_NOP, 1);
  EXPECT_EQ(rec2.ptg_last_k_NOP, 7);
  EXPECT_NEAR(rec2.rel_cur_pose_wrt_last_vel_cmd_NOP.x, 0.1, 1e-9);
  EXPECT_NEAR(rec2.rel_pose_PTG_origin_wrt_sense_NOP.y, 0.5, 1e-9);
  EXPECT_FALSE(rec2.cmd_vel);
}

TEST(CLogFileRecord, record_without_a_cmd_vel_roundtrips)
{
  CLogFileRecord rec;
  rec.nPTGs = 0;
  rec.ptg_index_NOP = -1;  // NOP mode disabled: the NOP fields are skipped

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << rec;
  buf.Seek(0);

  CLogFileRecord rec2;
  arch >> rec2;
  EXPECT_EQ(rec2.nPTGs, 0U);
  EXPECT_FALSE(rec2.cmd_vel);
  EXPECT_EQ(rec2.ptg_index_NOP, -1);
}
