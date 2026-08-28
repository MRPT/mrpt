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

/** Unit tests for the smaller pieces of mrpt_nav: the clearance diagram,
 *  the geometric collision helpers, the default CRobot2NavInterface
 *  implementations, the navigation logger, the velocity filter and the
 *  multi-objective motion optimizers.
 */

#include <gtest/gtest.h>
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/kinematics/CVehicleVelCmd_DiffDriven.h>
#include <mrpt/nav/holonomic/CHolonomicVFF.h>
#include <mrpt/nav/holonomic/ClearanceDiagram.h>
#include <mrpt/nav/planners/PlannerSimple2D.h>
#include <mrpt/nav/planners/nav_plan_geometry_utils.h>
#include <mrpt/nav/reactive/CMultiObjMotionOpt_Scalarization.h>
#include <mrpt/nav/reactive/CRobot2NavInterface.h>
#include <mrpt/nav/reactive/NavigationLogger.h>
#include <mrpt/nav/reactive/VelocityFilter.h>
#include <mrpt/nav/tpspace/CPTG_DiffDrive_C.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/viz/CMesh.h>

#include <cmath>
#include <fstream>

using namespace mrpt::nav;

// ---------------------------------------------------------------------------
//  ClearanceDiagram
// ---------------------------------------------------------------------------
namespace
{
ClearanceDiagram make_clearance_diagram(size_t nPaths = 20, size_t nDecim = 5)
{
  ClearanceDiagram cd;
  cd.resize(nPaths, nDecim);
  for (size_t k = 0; k < nDecim; k++)
  {
    auto& m = cd.get_path_clearance_decimated(k);
    m[0.25] = 0.5;
    m[0.50] = 0.4;
    m[1.00] = 0.3;
  }
  return cd;
}
}  // namespace

TEST(ClearanceDiagram, resize_maps_between_real_and_decimated_indices)
{
  auto cd = make_clearance_diagram(21, 6);
  EXPECT_EQ(cd.get_actual_num_paths(), 21U);
  EXPECT_EQ(cd.get_decimated_num_paths(), 6U);

  EXPECT_EQ(cd.real_k_to_decimated_k(0), 0U);
  EXPECT_EQ(cd.real_k_to_decimated_k(20), 5U);
  EXPECT_EQ(cd.decimated_k_to_real_k(0), 0U);
  EXPECT_EQ(cd.decimated_k_to_real_k(5), 20U);

  // Out-of-range indices are rejected:
  EXPECT_ANY_THROW(cd.real_k_to_decimated_k(1000));
  EXPECT_ANY_THROW(cd.decimated_k_to_real_k(1000));
}

TEST(ClearanceDiagram, index_mapping_needs_a_non_empty_diagram)
{
  ClearanceDiagram cd;
  EXPECT_TRUE(cd.empty());
  EXPECT_ANY_THROW(cd.real_k_to_decimated_k(0));
  EXPECT_ANY_THROW(cd.decimated_k_to_real_k(0));
}

TEST(ClearanceDiagram, resize_to_zero_decimated_paths_clears_it)
{
  auto cd = make_clearance_diagram();
  ASSERT_FALSE(cd.empty());
  cd.resize(10, 0);
  EXPECT_TRUE(cd.empty());
  EXPECT_EQ(cd.get_actual_num_paths(), 0U);
}

TEST(ClearanceDiagram, resize_rejects_more_decimated_than_actual_paths)
{
  ClearanceDiagram cd;
  EXPECT_ANY_THROW(cd.resize(4, 8));
}

TEST(ClearanceDiagram, getClearance_of_an_empty_diagram_is_zero)
{
  ClearanceDiagram cd;
  EXPECT_EQ(cd.getClearance(0, 1.0, false), .0);
  EXPECT_EQ(cd.getClearance(0, 1.0, true), .0);
}

TEST(ClearanceDiagram, getClearance_averages_or_integrates_over_the_path)
{
  auto cd = make_clearance_diagram();

  // Averaged over the path up to (and including) the first sample past the
  // query distance:
  const double avg = cd.getClearance(0, 0.5, false /*integrate_over_path*/);
  EXPECT_NEAR(avg, (0.5 + 0.4 + 0.3) / 3, 1e-9);

  // "integrate_over_path" keeps only the last visited sample:
  const double last = cd.getClearance(0, 0.5, true);
  EXPECT_NEAR(last, 0.3, 1e-9);

  // A query beyond every stored sample averages them all:
  EXPECT_NEAR(cd.getClearance(0, 100.0, false), (0.5 + 0.4 + 0.3) / 3, 1e-9);

  EXPECT_ANY_THROW(cd.getClearance(1000, 0.5, false));
}

TEST(ClearanceDiagram, path_clearance_accessors)
{
  auto cd = make_clearance_diagram(20, 5);
  const auto& constCd = cd;

  EXPECT_EQ(cd.get_path_clearance(0).size(), 3U);
  EXPECT_EQ(constCd.get_path_clearance(0).size(), 3U);
  EXPECT_EQ(constCd.get_path_clearance_decimated(0).size(), 3U);

  cd.get_path_clearance(0)[2.0] = 0.2;
  EXPECT_EQ(constCd.get_path_clearance(0).size(), 4U);
}

TEST(ClearanceDiagram, serialization_roundtrip)
{
  auto cd = make_clearance_diagram(20, 5);

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  cd.writeToStream(arch);
  buf.Seek(0);

  ClearanceDiagram cd2;
  cd2.readFromStream(arch);

  EXPECT_EQ(cd2.get_actual_num_paths(), cd.get_actual_num_paths());
  EXPECT_EQ(cd2.get_decimated_num_paths(), cd.get_decimated_num_paths());
  EXPECT_NEAR(cd2.getClearance(0, 1.0, false), cd.getClearance(0, 1.0, false), 1e-12);
}

TEST(ClearanceDiagram, clear_resets_everything)
{
  auto cd = make_clearance_diagram();
  cd.clear();
  EXPECT_TRUE(cd.empty());
  EXPECT_EQ(cd.get_actual_num_paths(), 0U);
  EXPECT_EQ(cd.get_decimated_num_paths(), 0U);
}

TEST(ClearanceDiagram, renderAs3DObject_fills_a_mesh)
{
  auto cd = make_clearance_diagram();

  mrpt::viz::CMesh mesh;
  cd.renderAs3DObject(mesh, -1.0, 1.0, -1.0, 1.0, 0.25, false);
  EXPECT_GT(mesh.getxMax(), mesh.getxMin());

  // An empty diagram renders nothing, but must not throw:
  ClearanceDiagram empty;
  mrpt::viz::CMesh mesh2;
  EXPECT_NO_THROW(empty.renderAs3DObject(mesh2, -1.0, 1.0, -1.0, 1.0, 0.25, true));

  // Degenerate rendering bounds are rejected:
  EXPECT_ANY_THROW(cd.renderAs3DObject(mesh, -1.0, 1.0, -1.0, 1.0, .0 /*cell_res*/, false));
  EXPECT_ANY_THROW(cd.renderAs3DObject(mesh, 1.0, -1.0, -1.0, 1.0, 0.25, false));
  EXPECT_ANY_THROW(cd.renderAs3DObject(mesh, -1.0, 1.0, 1.0, -1.0, 0.25, false));
}

// ---------------------------------------------------------------------------
//  nav_plan_geometry_utils
// ---------------------------------------------------------------------------
TEST(NavGeomUtils, straight_segment_hits_an_obstacle_ahead)
{
  double d = .0;
  // Robot of radius 0.5 moving along +x; obstacle at (2,0):
  EXPECT_TRUE(collision_free_dist_segment_circ_robot({0, 0}, {5, 0}, 0.5, {2, 0}, d));
  EXPECT_NEAR(d, 1.5, 1e-9);
}

TEST(NavGeomUtils, straight_segment_misses_a_lateral_obstacle)
{
  double d = .0;
  EXPECT_FALSE(collision_free_dist_segment_circ_robot({0, 0}, {5, 0}, 0.5, {2, 3}, d));
  EXPECT_LT(d, .0);
}

TEST(NavGeomUtils, straight_segment_ignores_obstacles_behind_and_beyond)
{
  double d = .0;
  // Behind the start point:
  EXPECT_FALSE(collision_free_dist_segment_circ_robot({0, 0}, {5, 0}, 0.5, {-3, 0}, d));
  // Beyond the segment end:
  EXPECT_FALSE(collision_free_dist_segment_circ_robot({0, 0}, {1, 0}, 0.5, {9, 0}, d));
}

TEST(NavGeomUtils, straight_segment_requires_a_non_degenerate_segment)
{
  double d = .0;
  EXPECT_ANY_THROW(collision_free_dist_segment_circ_robot({0, 0}, {0, 0}, 0.5, {2, 0}, d));
}

TEST(NavGeomUtils, arc_path_hits_an_obstacle_on_the_arc)
{
  // Arc turning left with radius 2 m: the trajectory passes through (2,2).
  double d = .0;
  const bool hit = collision_free_dist_arc_circ_robot(2.0, 0.5, {2.0, 2.0}, d);
  ASSERT_TRUE(hit);
  EXPECT_GT(d, .0);
  // A quarter of the circle is pi/2*R ~= 3.14 m; the collision happens a bit
  // earlier because of the robot radius:
  EXPECT_LT(d, 2.0 * M_PI / 2);
}

TEST(NavGeomUtils, arc_path_misses_a_far_away_obstacle)
{
  double d = .0;
  EXPECT_FALSE(collision_free_dist_arc_circ_robot(2.0, 0.5, {50.0, 50.0}, d));
  EXPECT_LT(d, .0);
}

TEST(NavGeomUtils, arc_path_misses_an_obstacle_inside_the_turning_circle)
{
  double d = .0;
  // The center of the turning circle is at (0,2): an obstacle right there is
  // never touched by the arc.
  EXPECT_FALSE(collision_free_dist_arc_circ_robot(2.0, 0.1, {0.0, 2.0}, d));
}

TEST(NavGeomUtils, arc_path_works_for_right_turns_too)
{
  double d = .0;
  const bool hit = collision_free_dist_arc_circ_robot(-2.0, 0.5, {2.0, -2.0}, d);
  ASSERT_TRUE(hit);
  EXPECT_GT(d, .0);
}

TEST(NavGeomUtils, arc_path_requires_a_non_degenerate_radius)
{
  double d = .0;
  EXPECT_ANY_THROW(collision_free_dist_arc_circ_robot(.0, 0.5, {2.0, 2.0}, d));
}

// ---------------------------------------------------------------------------
//  CRobot2NavInterface default implementations
// ---------------------------------------------------------------------------
namespace
{
/** Implements only the pure virtual methods, leaving every optional callback
 *  at its base-class default. */
struct BareRobotIF : public CRobot2NavInterface
{
  std::optional<CurrentPoseAndSpeeds> getCurrentPoseAndSpeeds() override
  {
    return CurrentPoseAndSpeeds();
  }
  bool changeSpeeds(const mrpt::kinematics::CVehicleVelCmd&) override { return true; }
  bool stop(StopType) override { return true; }
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
};
}  // namespace

TEST(CRobot2NavInterface, default_callbacks_are_harmless_no_ops)
{
  BareRobotIF r;
  r.setMinLoggingLevel(mrpt::system::LVL_ERROR);

  EXPECT_TRUE(r.changeSpeedsNOP());
  EXPECT_TRUE(r.startWatchdog(1000.f));
  EXPECT_TRUE(r.stopWatchdog());
  EXPECT_FALSE(r.getAlignCmd(1.0));  // in-place rotations unsupported

  EXPECT_NO_THROW(r.sendNavigationStartEvent());
  EXPECT_NO_THROW(r.sendNavigationEndEvent());
  EXPECT_NO_THROW(r.sendWaypointReachedEvent(0, true));
  EXPECT_NO_THROW(r.sendWaypointReachedEvent(0, false));
  EXPECT_NO_THROW(r.sendNewWaypointTargetEvent(1));
  EXPECT_NO_THROW(r.sendNavigationEndDueToErrorEvent());
  EXPECT_NO_THROW(r.sendWaySeemsBlockedEvent());
  EXPECT_NO_THROW(r.sendApparentCollisionEvent());
  EXPECT_NO_THROW(r.sendCannotGetCloserToBlockedTargetEvent());
}

TEST(CRobot2NavInterface, navigation_timer_starts_from_zero)
{
  BareRobotIF r;
  r.resetNavigationTimer();
  const double t = r.getNavigationTime();
  EXPECT_GE(t, .0);
  EXPECT_LT(t, 5.0);
}

// ---------------------------------------------------------------------------
//  NavigationLogger
// ---------------------------------------------------------------------------
TEST(NavigationLogger, log_directory_accessors)
{
  NavigationLogger lg;
  EXPECT_FALSE(lg.getLogFileDirectory().empty());

  lg.setLogFileDirectory("./some/dir");
  EXPECT_EQ(lg.getLogFileDirectory(), "./some/dir");
}

TEST(NavigationLogger, keeping_records_in_memory)
{
  NavigationLogger lg;
  EXPECT_FALSE(lg.shouldFillLogRecord());

  lg.enableKeepLogRecords(true);
  EXPECT_TRUE(lg.shouldFillLogRecord());

  CLogFileRecord rec;
  rec.nSelectedPTG = 3;
  rec.robotPoseLocalization = mrpt::math::TPose2D(1, 2, 0.3);

  mrpt::system::CTimeLogger tl(false);
  lg.writeLogRecord(rec, tl);

  CLogFileRecord out;
  lg.getLastLogRecord(out);
  EXPECT_EQ(out.nSelectedPTG, 3);
  EXPECT_NEAR(out.robotPoseLocalization.x, 1.0, 1e-9);

  lg.enableKeepLogRecords(false);
  EXPECT_FALSE(lg.shouldFillLogRecord());
}

TEST(NavigationLogger, writes_and_closes_a_log_file)
{
  const std::string dir = mrpt::system::getTempFileName() + std::string("_navlog");

  NavigationLogger lg;
  EXPECT_FALSE(lg.isNewLogFile());
  EXPECT_EQ(lg.getLogStream(), nullptr);

  lg.enableLogFile(true, dir, nullptr);
  ASSERT_NE(lg.getLogStream(), nullptr);
  EXPECT_TRUE(lg.shouldFillLogRecord());
  EXPECT_TRUE(lg.isNewLogFile());

  lg.markLogFileIntroduced();
  EXPECT_FALSE(lg.isNewLogFile());

  CLogFileRecord rec;
  mrpt::system::CTimeLogger tl(false);
  lg.writeLogRecord(rec, tl);

  // Enabling twice is a no-op:
  auto* prevStream = lg.getLogStream();
  lg.enableLogFile(true, dir, nullptr);
  EXPECT_EQ(lg.getLogStream(), prevStream);

  EXPECT_TRUE(mrpt::system::fileExists(dir + "/log_000.reactivenavlog"));

  lg.enableLogFile(false, dir, nullptr);
  EXPECT_EQ(lg.getLogStream(), nullptr);

  // Disabling twice is also a no-op:
  EXPECT_NO_THROW(lg.enableLogFile(false, dir, nullptr));

  lg.close();
  EXPECT_EQ(lg.getLogStream(), nullptr);

  mrpt::system::deleteFilesInDirectory(dir, true);
}

TEST(NavigationLogger, an_unusable_log_directory_is_reported_not_thrown)
{
  NavigationLogger lg;
  // A path that cannot be created (a component of it is an existing file):
  const std::string fil = mrpt::system::getTempFileName();
  {
    std::ofstream f(fil);
    f << "x";
  }
  EXPECT_NO_THROW(lg.enableLogFile(true, fil + "/sub", nullptr));
  EXPECT_EQ(lg.getLogStream(), nullptr);
  mrpt::system::deleteFile(fil);
}

// ---------------------------------------------------------------------------
//  VelocityFilter
// ---------------------------------------------------------------------------
namespace
{
CParameterizedTrajectoryGenerator::Ptr make_test_ptg()
{
  mrpt::config::CConfigFileMemory cfg;
  cfg.write("PTG", "num_paths", 21);
  cfg.write("PTG", "refDistance", 2.0);
  cfg.write("PTG", "resolution", 0.25);
  cfg.write("PTG", "v_max_mps", 1.0);
  cfg.write("PTG", "w_max_dps", 60.0);
  cfg.write("PTG", "K", 1.0);
  cfg.write("PTG", "shape_x0", -0.2);
  cfg.write("PTG", "shape_y0", 0.2);
  cfg.write("PTG", "shape_x1", 0.2);
  cfg.write("PTG", "shape_y1", 0.2);
  cfg.write("PTG", "shape_x2", 0.2);
  cfg.write("PTG", "shape_y2", -0.2);
  cfg.write("PTG", "shape_x3", -0.2);
  cfg.write("PTG", "shape_y3", -0.2);
  auto ptg = CParameterizedTrajectoryGenerator::CreatePTG("CPTG_DiffDrive_C", cfg, "PTG", "");
  ptg->initialize(std::string(), false);
  return ptg;
}
}  // namespace

TEST(VelocityFilter, zero_speed_produces_a_stop_command)
{
  VelocityFilter vf;
  mrpt::system::CTimeLogger tl(false);

  TCandidateMovementPTG mov;
  auto ptgTmp = make_test_ptg();
  mov.PTG = ptgTmp.get();
  mov.speed = .0;
  mov.direction = .0;

  mrpt::kinematics::CVehicleVelCmd::Ptr cmd;
  mrpt::kinematics::CVehicleVelCmd::TVelCmdParams params;
  params.robotMax_V_mps = 1.0;
  params.robotMax_W_radps = 1.0;

  const double scale = vf.generateVelCmd(mov, cmd, .0, params, 0.1, tl);
  ASSERT_TRUE(cmd);
  EXPECT_TRUE(cmd->isStopCmd());
  EXPECT_NEAR(scale, 1.0, 1e-12);
}

TEST(VelocityFilter, non_zero_speed_scales_and_remembers_the_last_command)
{
  VelocityFilter vf;
  mrpt::system::CTimeLogger tl(false);
  auto ptg = make_test_ptg();

  TCandidateMovementPTG mov;
  mov.PTG = ptg.get();
  mov.speed = 0.5;
  mov.direction = .0;

  mrpt::kinematics::CVehicleVelCmd::Ptr cmd;
  mrpt::kinematics::CVehicleVelCmd::TVelCmdParams params;
  params.robotMax_V_mps = 1.0;
  params.robotMax_W_radps = 1.0;

  EXPECT_FALSE(vf.getLastVelCmd());
  const double scale = vf.generateVelCmd(mov, cmd, .0, params, 0.1, tl);
  ASSERT_TRUE(cmd);
  EXPECT_GT(scale, .0);
  EXPECT_LE(scale, 1.0);
  EXPECT_FALSE(cmd->isStopCmd());
  EXPECT_TRUE(vf.getLastVelCmd());

  vf.resetLastVelCmd();
  EXPECT_FALSE(vf.getLastVelCmd());
}

TEST(VelocityFilter, slowdown_movements_are_not_rescaled)
{
  VelocityFilter vf;
  mrpt::system::CTimeLogger tl(false);
  auto ptg = make_test_ptg();

  TCandidateMovementPTG mov;
  mov.PTG = ptg.get();
  mov.speed = 0.5;
  mov.direction = .0;
  mov.props["is_slowdown"] = 1.0;

  mrpt::kinematics::CVehicleVelCmd::Ptr cmd;
  mrpt::kinematics::CVehicleVelCmd::TVelCmdParams params;
  params.robotMax_V_mps = 1.0;
  params.robotMax_W_radps = 1.0;

  const double scale = vf.generateVelCmd(mov, cmd, .0, params, 0.1, tl);
  ASSERT_TRUE(cmd);
  EXPECT_NEAR(scale, 1.0, 1e-12);
}

// ---------------------------------------------------------------------------
//  Multi-objective motion optimizers
// ---------------------------------------------------------------------------
namespace
{
TCandidateMovementPTG make_candidate(double speed, double collision_free_distance)
{
  TCandidateMovementPTG m;
  m.speed = speed;
  m.direction = .0;
  m.props["collision_free_distance"] = collision_free_distance;
  m.props["hysteresis"] = 0.0;
  m.props["clearance"] = 1.0;
  m.props["dist_eucl_final"] = 1.0;
  m.props["target_k"] = 0.0;
  m.props["move_k"] = 0.0;
  m.props["num_paths"] = 100.0;
  m.props["ref_dist"] = 10.0;
  return m;
}
}  // namespace

TEST(MultiObjOpt, factory_creates_known_optimizers_only)
{
  auto opt = CMultiObjectiveMotionOptimizerBase::Factory("CMultiObjMotionOpt_Scalarization");
  EXPECT_TRUE(opt);
  EXPECT_FALSE(CMultiObjectiveMotionOptimizerBase::Factory("NoSuchOptimizerClass"));
  // A registered class that is not a motion optimizer:
  EXPECT_FALSE(CMultiObjectiveMotionOptimizerBase::Factory("mrpt::poses::CPose3D"));
}

TEST(MultiObjOpt, default_params_define_the_standard_scores)
{
  CMultiObjectiveMotionOptimizerBase::TParamsBase p;
  EXPECT_FALSE(p.formula_score.empty());
  EXPECT_NE(p.formula_score.find("collision_free_distance"), p.formula_score.end());
  EXPECT_NE(p.formula_score.find("hysteresis"), p.formula_score.end());
  EXPECT_EQ(p.scores_to_normalize.size(), 1U);
  EXPECT_EQ(p.scores_to_normalize[0], "clearance");
}

TEST(MultiObjOpt, params_config_file_roundtrip)
{
  CMultiObjMotionOpt_Scalarization opt;
  opt.parameters.formula_score.clear();
  opt.parameters.formula_score["s1"] = "collision_free_distance";
  opt.parameters.formula_score["s2"] = "clearance";
  opt.parameters.movement_assert.emplace_back("collision_free_distance>0.1");
  opt.parameters.scores_to_normalize = {"s2"};
  opt.parameters.scalar_score_formula = "s1+s2";

  mrpt::config::CConfigFileMemory cfg;
  opt.saveConfigFile(cfg);

  CMultiObjMotionOpt_Scalarization opt2;
  opt2.loadConfigFile(cfg);

  EXPECT_EQ(opt2.parameters.formula_score.size(), 2U);
  EXPECT_EQ(opt2.parameters.formula_score["s1"], "collision_free_distance");
  ASSERT_EQ(opt2.parameters.movement_assert.size(), 1U);
  EXPECT_EQ(opt2.parameters.movement_assert[0], "collision_free_distance>0.1");
  ASSERT_EQ(opt2.parameters.scores_to_normalize.size(), 1U);
  EXPECT_EQ(opt2.parameters.scores_to_normalize[0], "s2");
  EXPECT_EQ(opt2.parameters.scalar_score_formula, "s1+s2");
}

TEST(MultiObjOpt, params_load_rejects_incomplete_score_definitions)
{
  {
    mrpt::config::CConfigFileMemory cfg;
    cfg.write("S", "dummy", "");  // section exists, but no scores at all
    CMultiObjectiveMotionOptimizerBase::TParamsBase p;
    EXPECT_ANY_THROW(p.loadFromConfigFile(cfg, "S"));
  }
  {
    mrpt::config::CConfigFileMemory cfg;
    cfg.write("S", "score1_name", "a");  // name without formula
    CMultiObjectiveMotionOptimizerBase::TParamsBase p;
    EXPECT_ANY_THROW(p.loadFromConfigFile(cfg, "S"));
  }
}

TEST(MultiObjOpt, movement_asserts_veto_candidates)
{
  CMultiObjMotionOpt_Scalarization opt;
  opt.parameters.formula_score.clear();
  opt.parameters.formula_score["s"] = "collision_free_distance";
  opt.parameters.scores_to_normalize.clear();
  opt.parameters.movement_assert.emplace_back("collision_free_distance>0.5");
  opt.parameters.scalar_score_formula = "s";

  std::vector<TCandidateMovementPTG> movs{
      make_candidate(1.0, 0.2 /*vetoed*/), make_candidate(1.0, 0.9 /*accepted*/)};

  CMultiObjectiveMotionOptimizerBase::TResultInfo info;
  const auto best = opt.decide(movs, info);
  ASSERT_TRUE(best.has_value());
  EXPECT_EQ(*best, 1U);
  EXPECT_FALSE(info.log_entries.empty());
}

TEST(MultiObjOpt, scores_are_normalized_when_requested)
{
  CMultiObjMotionOpt_Scalarization opt;
  opt.parameters.formula_score.clear();
  opt.parameters.formula_score["s"] = "collision_free_distance";
  opt.parameters.scores_to_normalize = {"s"};
  opt.parameters.scalar_score_formula = "s";

  std::vector<TCandidateMovementPTG> movs{make_candidate(1.0, 0.5), make_candidate(1.0, 2.0)};

  CMultiObjectiveMotionOptimizerBase::TResultInfo info;
  const auto best = opt.decide(movs, info);
  ASSERT_TRUE(best.has_value());
  EXPECT_EQ(*best, 1U);
  // The maximum score is normalized to exactly 1.0:
  EXPECT_NEAR(info.score_values[1]["s"], 1.0, 1e-9);
  EXPECT_NEAR(info.score_values[0]["s"], 0.25, 1e-9);
}

TEST(MultiObjOpt, all_zero_scores_are_normalized_to_one)
{
  CMultiObjMotionOpt_Scalarization opt;
  opt.parameters.formula_score.clear();
  opt.parameters.formula_score["s"] = "collision_free_distance";
  opt.parameters.scores_to_normalize = {"s"};
  opt.parameters.scalar_score_formula = "s";

  std::vector<TCandidateMovementPTG> movs{make_candidate(1.0, .0), make_candidate(1.0, .0)};

  CMultiObjectiveMotionOptimizerBase::TResultInfo info;
  const auto best = opt.decide(movs, info);
  ASSERT_TRUE(best.has_value());
  EXPECT_NEAR(info.score_values[0]["s"], 1.0, 1e-9);
  EXPECT_NEAR(info.score_values[1]["s"], 1.0, 1e-9);
}

TEST(MultiObjOpt, invalid_candidates_score_zero)
{
  CMultiObjMotionOpt_Scalarization opt;
  opt.parameters.formula_score.clear();
  opt.parameters.formula_score["s"] = "collision_free_distance";
  opt.parameters.scores_to_normalize.clear();
  opt.parameters.scalar_score_formula = "s";

  std::vector<TCandidateMovementPTG> movs{
      make_candidate(-1.0 /*inviable*/, 5.0), make_candidate(1.0, 1.0)};

  CMultiObjectiveMotionOptimizerBase::TResultInfo info;
  const auto best = opt.decide(movs, info);
  ASSERT_TRUE(best.has_value());
  EXPECT_EQ(*best, 1U);
  EXPECT_NEAR(info.score_values[0]["s"], .0, 1e-12);
}

TEST(MultiObjOpt, a_score_formula_that_cannot_be_compiled_yields_no_decision)
{
  CMultiObjMotionOpt_Scalarization opt;
  opt.parameters.formula_score.clear();
  opt.parameters.formula_score["s"] = "this is ***not*** a valid expression";
  opt.parameters.scores_to_normalize.clear();
  opt.parameters.scalar_score_formula = "s";

  std::vector<TCandidateMovementPTG> movs{make_candidate(1.0, 1.0)};

  CMultiObjectiveMotionOptimizerBase::TResultInfo info;
  const auto best = opt.decide(movs, info);
  EXPECT_FALSE(best.has_value());
}

TEST(MultiObjOpt, a_failed_compilation_leaves_no_stale_state_behind)
{
  CMultiObjMotionOpt_Scalarization opt;
  opt.parameters.formula_score.clear();
  // The first score compiles, the second does not: the names registered while
  // compiling the first must not survive into the next decide().
  opt.parameters.formula_score["aaa_ok"] = "collision_free_distance";
  opt.parameters.formula_score["zzz_bad"] = "***not valid***";
  opt.parameters.scores_to_normalize.clear();
  opt.parameters.scalar_score_formula = "aaa_ok";

  std::vector<TCandidateMovementPTG> movs{make_candidate(1.0, 1.0)};

  CMultiObjectiveMotionOptimizerBase::TResultInfo info;
  EXPECT_FALSE(opt.decide(movs, info).has_value());

  // Repairing the formula must make the optimizer usable again:
  opt.parameters.formula_score["zzz_bad"] = "clearance";
  CMultiObjectiveMotionOptimizerBase::TResultInfo info2;
  EXPECT_TRUE(opt.decide(movs, info2).has_value());
}

TEST(MultiObjOpt, a_movement_assert_that_cannot_be_compiled_yields_no_decision)
{
  CMultiObjMotionOpt_Scalarization opt;
  opt.parameters.formula_score.clear();
  opt.parameters.formula_score["s"] = "collision_free_distance";
  opt.parameters.scores_to_normalize.clear();
  opt.parameters.movement_assert.emplace_back("***not valid***");
  opt.parameters.scalar_score_formula = "s";

  std::vector<TCandidateMovementPTG> movs{make_candidate(1.0, 1.0)};

  CMultiObjectiveMotionOptimizerBase::TResultInfo info;
  const auto best = opt.decide(movs, info);
  EXPECT_FALSE(best.has_value());
}

TEST(MultiObjOpt, a_score_name_clashing_with_an_input_variable_is_rejected)
{
  CMultiObjMotionOpt_Scalarization opt;
  opt.parameters.formula_score.clear();
  // "hysteresis" is one of the per-candidate input variables:
  opt.parameters.formula_score["hysteresis"] = "collision_free_distance";
  opt.parameters.scores_to_normalize.clear();
  opt.parameters.scalar_score_formula = "hysteresis";

  std::vector<TCandidateMovementPTG> movs{make_candidate(1.0, 1.0)};

  CMultiObjectiveMotionOptimizerBase::TResultInfo info;
  EXPECT_ANY_THROW((void)opt.decide(movs, info));
}

TEST(MultiObjOpt, clear_forces_the_formulas_to_be_recompiled)
{
  CMultiObjMotionOpt_Scalarization opt;
  opt.parameters.formula_score.clear();
  opt.parameters.formula_score["s"] = "collision_free_distance";
  opt.parameters.scores_to_normalize.clear();
  opt.parameters.scalar_score_formula = "s";

  std::vector<TCandidateMovementPTG> movs{make_candidate(1.0, 1.0)};
  CMultiObjectiveMotionOptimizerBase::TResultInfo info;
  ASSERT_TRUE(opt.decide(movs, info).has_value());

  opt.parameters.scalar_score_formula = "2*s";
  opt.clear();

  CMultiObjectiveMotionOptimizerBase::TResultInfo info2;
  const auto best = opt.decide(movs, info2);
  ASSERT_TRUE(best.has_value());
  EXPECT_NEAR(info2.final_evaluation[0], 2 * info.final_evaluation[0], 1e-9);
}

TEST(MultiObjOpt, an_invalid_scalar_formula_throws)
{
  CMultiObjMotionOpt_Scalarization opt;
  opt.parameters.formula_score.clear();
  opt.parameters.formula_score["s"] = "collision_free_distance";
  opt.parameters.scores_to_normalize.clear();
  opt.parameters.scalar_score_formula = "***nope***";

  std::vector<TCandidateMovementPTG> movs{make_candidate(1.0, 1.0)};
  CMultiObjectiveMotionOptimizerBase::TResultInfo info;
  EXPECT_ANY_THROW((void)opt.decide(movs, info));
}

// ---------------------------------------------------------------------------
//  A few remaining corners
// ---------------------------------------------------------------------------
TEST(CHolonomicVFF, log_record_serialization_roundtrip)
{
  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);

  CLogFileRecord_VFF rec;
  arch << rec;
  buf.Seek(0);

  CLogFileRecord_VFF rec2;
  EXPECT_NO_THROW(arch >> rec2);
}

TEST(PlannerSimple2D, out_of_grid_endpoints_are_reported_as_not_found)
{
  mrpt::maps::COccupancyGridMap2D grid;
  grid.setSize(-5.0f, 5.0f, -5.0f, 5.0f, 0.10f);
  grid.fill(0.6f);

  PlannerSimple2D planner;
  planner.robotRadius = 0.15f;

  const mrpt::poses::CPose2D inside(0, 0, 0), outside(100, 100, 0);
  std::deque<mrpt::math::TPoint2D> path;
  bool notFound = false;

  planner.computePath(grid, outside, inside, path, notFound);
  EXPECT_TRUE(notFound);

  notFound = false;
  planner.computePath(grid, inside, outside, path, notFound);
  EXPECT_TRUE(notFound);

  notFound = false;
  planner.computePath(grid, outside, outside, path, notFound);
  EXPECT_TRUE(notFound);
}

TEST(PlannerSimple2D, origin_and_target_in_the_same_cell)
{
  mrpt::maps::COccupancyGridMap2D grid;
  grid.setSize(-5.0f, 5.0f, -5.0f, 5.0f, 0.10f);
  grid.fill(0.6f);

  PlannerSimple2D planner;
  std::deque<mrpt::math::TPoint2D> path;
  bool notFound = true;

  planner.computePath(
      grid, mrpt::poses::CPose2D(0.01, 0.01, 0), mrpt::poses::CPose2D(0.02, 0.02, 0), path,
      notFound);

  EXPECT_FALSE(notFound);
  ASSERT_EQ(path.size(), 1U);
  EXPECT_NEAR(path.front().x, 0.02, 1e-9);
}
