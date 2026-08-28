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

/** Reactive navigation runs driven by fully in-memory configurations, so they
 *  do not depend on the shared config files. They exercise the optional
 *  features of CAbstractPTGBasedReactive that the default configuration used
 *  by rnav_unittest.cpp leaves switched off: the delays model, clearance
 *  evaluation, velocity filtering, "NOP cmdvel" PTG continuation, obstacle
 *  filtering, log records and the visualization helpers.
 */

#include <gtest/gtest.h>
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/kinematics/CVehicleSimul_Holo.h>
#include <mrpt/nav/reactive/CReactiveNavigationSystem.h>
#include <mrpt/nav/reactive/CRobot2NavInterfaceForSimulator.h>
#include <mrpt/system/filesystem.h>

#include <string>

using namespace mrpt::nav;

namespace
{
/** A holonomic simulated robot that reports a fixed set of obstacles. */
struct HoloSimRobot : public CRobot2NavInterfaceForSimulator_Holo
{
  std::vector<mrpt::math::TPoint2D> obstacles;
  bool senseSucceeds = true;

  HoloSimRobot(mrpt::kinematics::CVehicleSimul_Holo& sim) :
      CRobot2NavInterfaceForSimulator_Holo(sim)
  {
    this->setMinLoggingLevel(mrpt::system::LVL_ERROR);
  }

  void sendNavigationStartEvent() override {}
  void sendNavigationEndEvent() override {}

  bool senseObstacles(
      mrpt::maps::CSimplePointsMap& obs, mrpt::system::TTimeStamp& timestamp) override
  {
    obs.clear();
    timestamp = mrpt::Clock::now();
    if (!senseSucceeds) return false;
    for (const auto& p : obstacles) obs.insertPoint(p.x, p.y, 0.0);
    return true;
  }
};

/** Builds an in-memory reactive-navigation config. `holoPTG` selects a
 *  CPTG_Holo_Blend (which supports "NOP cmdvel" PTG continuation) instead of
 *  the differential-drive default. */
std::string make_rnav_config(
    bool useDelaysModel, bool evaluateClearance, double speedFilterTau, bool obstacleFiltering)
{
  std::string s;
  s += "[CAbstractNavigator]\n";
  s += "dist_to_target_for_sending_event = 0\n";
  s += "alarm_seems_not_approaching_target_timeout = 30\n";
  s += "dist_check_target_is_blocked = 0.5\n";
  s += "hysteresis_check_target_is_blocked = 3\n";
  s += "enable_time_profiler = false\n";

  s += "[CWaypointsNavigator]\n";
  s += "max_distance_to_allow_skip_waypoint = -1\n";
  s += "min_timesteps_confirm_skip_waypoints = 1\n";
  s += "waypoint_angle_tolerance = 5.0\n";
  s += "multitarget_look_ahead = 0\n";
  s += "minimum_target_approach_per_step = 0.02\n";

  s += "[CAbstractPTGBasedReactive]\n";
  s += "robotMax_V_mps = 1.0\n";
  s += "robotMax_W_degps = 60.0\n";
  s += "holonomic_method = CHolonomicVFF\n";
  s += "motion_decider_method = CMultiObjMotionOpt_Scalarization\n";
  s += "ref_distance = 4.0\n";
  s += "speedfilter_tau = " + std::to_string(speedFilterTau) + "\n";
  s += "secure_distance_start = 0.05\n";
  s += "secure_distance_end = 0.15\n";
  s += std::string("use_delays_model = ") + (useDelaysModel ? "true" : "false") + "\n";
  s += "max_distance_predicted_actual_path = 0.15\n";
  s += "min_normalized_free_space_for_ptg_continuation = 0.2\n";
  s += std::string("enable_obstacle_filtering = ") + (obstacleFiltering ? "true" : "false") + "\n";
  s += std::string("evaluate_clearance = ") + (evaluateClearance ? "true" : "false") + "\n";

  s += "[CPointCloudFilterByDistance]\n";
  s += "min_dist = 0.1\n";
  s += "angle_tolerance = 5.0\n";
  s += "too_old_seconds = 1.0\n";
  s += "previous_keyframes = 1\n";
  s += "max_deletion_ratio = 0.4\n";

  s += "[CHolonomicVFF]\n";
  s += "TARGET_SLOW_APPROACHING_DISTANCE = 0.10\n";
  s += "TARGET_ATTRACTIVE_FORCE = 20.0\n";

  s += "[CHolonomicND]\n";
  s += "WIDE_GAP_SIZE_PERCENT = 0.25\n";
  s += "MAX_SECTOR_DIST_FOR_D2_PERCENT = 0.25\n";
  s += "RISK_EVALUATION_SECTORS_PERCENT = 0.10\n";
  s += "RISK_EVALUATION_DISTANCE = 0.4\n";
  s += "TOO_CLOSE_OBSTACLE = 0.15\n";
  s += "TARGET_SLOW_APPROACHING_DISTANCE = 0.6\n";
  s += "factorWeights = 1.00 0.50 2.00 0.40\n";

  s += "[CMultiObjectiveMotionOptimizerBase]\n";
  s += "score1_name = collision_free_distance_score\n";
  s += "score1_formula = collision_free_distance\n";
  s += "score2_name = euclidean_nearness\n";
  s += "score2_formula = 1/(1+dist_eucl_min^2)\n";
  s += "scores_to_normalize = \n";

  s += "[CMultiObjMotionOpt_Scalarization]\n";
  s += "scalar_score_formula = 0.5*collision_free_distance_score + 8.0*euclidean_nearness\n";

  s += "[CReactiveNavigationSystem]\n";
  s += "min_obstacles_height = 0.0\n";
  s += "max_obstacles_height = 10.0\n";
  s += "PTG_COUNT = 1\n";
  s += "PTG0_Type = CPTG_Holo_Blend\n";
  s += "PTG0_refDistance = 4.0\n";
  s += "PTG0_num_paths = 31\n";
  s += "PTG0_v_max_mps = 1.0\n";
  s += "PTG0_w_max_dps = 60\n";
  s += "PTG0_T_ramp_max = 0.8\n";
  s += "PTG0_score_priority = 1.0\n";
  s += "RobotModel_circular_shape_radius = 0.3\n";
  return s;
}

struct RNavFixture
{
  mrpt::kinematics::CVehicleSimul_Holo simul;
  HoloSimRobot robot{simul};
  std::unique_ptr<CReactiveNavigationSystem> nav;

  void build(const std::string& cfgText)
  {
    nav = std::make_unique<CReactiveNavigationSystem>(robot, false /*no console output*/);
    nav->enableTimeLog(false);
    nav->setMinLoggingLevel(mrpt::system::LVL_ERROR);

    mrpt::config::CConfigFileMemory cfg(cfgText);
    nav->loadConfigFile(cfg);
    nav->initialize();
  }

  /** Runs the navigation loop under a simulated clock. */
  void run(const mrpt::math::TPoint2D& target, unsigned int maxIters = 60)
  {
    CAbstractNavigator::TNavigationParams np;
    np.target.target_coords = mrpt::math::TPose2D(target.x, target.y, .0);
    np.target.targetAllowedDistance = 0.4f;
    nav->navigate(&np);

    const auto savedClock = mrpt::Clock::getActiveClock();
    mrpt::Clock::setSimulatedTime(mrpt::Clock::now());
    mrpt::Clock::setActiveClock(mrpt::Clock::Simulated);

    const auto dt = std::chrono::milliseconds(200);
    for (unsigned int i = 0; i < maxIters; i++)
    {
      nav->navigationStep();
      if (nav->getCurrentState() == CAbstractNavigator::TState::IDLE) break;
      simul.simulateOneTimeStep(0.2);
      mrpt::Clock::setSimulatedTime(mrpt::Clock::now() + dt);
    }
    mrpt::Clock::setActiveClock(savedClock);

    using mrpt::system::CTimeLogger;
    const_cast<CTimeLogger&>(nav->getTimeLogger()).clear(true);
    const_cast<CTimeLogger&>(nav->getDelaysTimeLogger()).clear(true);
  }

  double distToTarget(const mrpt::math::TPoint2D& target) const
  {
    return (mrpt::math::TPoint2D(simul.getCurrentGTPose()) - target).norm();
  }
};
}  // namespace

TEST(RNavVariants, plain_run_reaches_the_target)
{
  RNavFixture f;
  f.build(make_rnav_config(false, false, .0, true));
  EXPECT_EQ(f.nav->getPTG_count(), 1U);
  ASSERT_NE(f.nav->getPTG(0), nullptr);
  EXPECT_TRUE(f.nav->getPTG(0)->supportVelCmdNOP());

  const mrpt::math::TPoint2D trg(2.0, 0.5);
  f.run(trg);
  EXPECT_LT(f.distToTarget(trg), 0.5);
  EXPECT_EQ(f.nav->getCurrentState(), CAbstractNavigator::TState::IDLE);
}

TEST(RNavVariants, delays_model_run)
{
  RNavFixture f;
  f.build(make_rnav_config(true /*use_delays_model*/, false, .0, true));

  const mrpt::math::TPoint2D trg(2.0, .0);
  f.run(trg);
  EXPECT_LT(f.distToTarget(trg), 0.6);
}

TEST(RNavVariants, clearance_evaluation_run)
{
  RNavFixture f;
  f.build(make_rnav_config(false, true /*evaluate_clearance*/, .0, true));
  f.robot.obstacles = {
      {1.0,  1.0},
      {1.0, -1.0},
      {1.5,  1.2}
  };

  const mrpt::math::TPoint2D trg(2.0, .0);
  f.run(trg);
  EXPECT_LT(f.distToTarget(trg), 0.6);
}

TEST(RNavVariants, velocity_filtering_run)
{
  RNavFixture f;
  f.build(make_rnav_config(false, false, 0.2 /*speedfilter_tau*/, true));

  const mrpt::math::TPoint2D trg(2.0, .0);
  f.run(trg);
  EXPECT_LT(f.distToTarget(trg), 0.6);
}

TEST(RNavVariants, obstacle_filtering_disabled_run)
{
  RNavFixture f;
  f.build(make_rnav_config(false, false, .0, false /*enable_obstacle_filtering*/));
  f.robot.obstacles = {
      {1.0,  1.0},
      {1.0, -1.0}
  };

  const mrpt::math::TPoint2D trg(2.0, .0);
  f.run(trg);
  EXPECT_LT(f.distToTarget(trg), 0.6);
}

TEST(RNavVariants, log_records_are_kept_and_written)
{
  RNavFixture f;
  f.build(make_rnav_config(false, false, .0, true));

  const std::string dir = mrpt::system::getTempFileName() + std::string("_rnavlog");
  f.nav->setLogFileDirectory(dir);
  f.nav->enableLogFile(true);
  f.nav->enableKeepLogRecords(true);

  const mrpt::math::TPoint2D trg(2.0, .0);
  f.run(trg, 15);

  CLogFileRecord rec;
  f.nav->getLastLogRecord(rec);
  EXPECT_GT(rec.nPTGs, 0U);
  EXPECT_FALSE(rec.infoPerPTG.empty());
  EXPECT_FALSE(rec.values.empty());

  f.nav->enableLogFile(false);
  EXPECT_TRUE(mrpt::system::fileExists(dir + "/log_000.reactivenavlog"));
  mrpt::system::deleteFilesInDirectory(dir, true);
}

TEST(RNavVariants, waypoint_navigation_run)
{
  RNavFixture f;
  f.build(make_rnav_config(false, false, .0, true));
  f.robot.obstacles = {
      {1.0,  1.0},
      {1.0, -1.0}
  };

  TWaypointSequence seq;
  // Note the explicit `speed_ratio`: with the default (1.0, i.e. "arrive at
  // full speed") this holonomic-PTG setup finds no viable movement at all.
  seq.waypoints.emplace_back(1.0, .0, 0.4, true, std::nullopt, 0.05 /*speed_ratio*/);
  seq.waypoints.emplace_back(2.0, .0, 0.4, true, std::nullopt, 0.05);
  f.nav->navigateWaypoints(seq);

  const auto savedClock = mrpt::Clock::getActiveClock();
  mrpt::Clock::setSimulatedTime(mrpt::Clock::now());
  mrpt::Clock::setActiveClock(mrpt::Clock::Simulated);

  const auto dt = std::chrono::milliseconds(200);
  for (unsigned int i = 0; i < 200; i++)
  {
    f.nav->navigationStep();
    if (f.nav->getWaypointNavStatus().final_goal_reached) break;
    f.simul.simulateOneTimeStep(0.2);
    mrpt::Clock::setSimulatedTime(mrpt::Clock::now() + dt);
  }
  mrpt::Clock::setActiveClock(savedClock);

  using mrpt::system::CTimeLogger;
  const_cast<CTimeLogger&>(f.nav->getTimeLogger()).clear(true);
  const_cast<CTimeLogger&>(f.nav->getDelaysTimeLogger()).clear(true);

  EXPECT_TRUE(f.nav->getWaypointNavStatus().final_goal_reached)
      << f.nav->getWaypointNavStatus().getAsText();
}

TEST(RNavVariants, restricting_the_usable_ptgs)
{
  RNavFixture f;
  f.build(make_rnav_config(false, false, .0, true));

  CAbstractPTGBasedReactive::TNavigationParamsPTG np;
  np.target.target_coords = mrpt::math::TPose2D(2.0, .0, .0);
  np.target.targetAllowedDistance = 0.4f;
  np.restrict_PTG_indices.push_back(0);

  EXPECT_NE(np.getAsText().find("restrict_PTG_indices"), std::string::npos);

  auto cloned = np.clone();
  ASSERT_TRUE(cloned);
  EXPECT_TRUE(np == *cloned);

  CAbstractPTGBasedReactive::TNavigationParamsPTG other;
  EXPECT_FALSE(np == other);

  f.nav->navigate(&np);
  EXPECT_NO_THROW(f.nav->navigationStep());
}

TEST(RNavVariants, a_failing_obstacle_sensor_stops_the_robot)
{
  RNavFixture f;
  f.build(make_rnav_config(false, false, .0, true));
  f.robot.senseSucceeds = false;

  CAbstractNavigator::TNavigationParams np;
  np.target.target_coords = mrpt::math::TPose2D(2.0, .0, .0);
  np.target.targetAllowedDistance = 0.4f;
  f.nav->navigate(&np);
  f.nav->navigationStep();

  EXPECT_EQ(f.nav->getCurrentState(), CAbstractNavigator::TState::NAV_ERROR);
}

TEST(RNavVariants, navigating_before_initialize_is_rejected)
{
  mrpt::kinematics::CVehicleSimul_Holo simul;
  HoloSimRobot robot{simul};
  CReactiveNavigationSystem nav(robot, false);
  nav.setMinLoggingLevel(mrpt::system::LVL_ERROR);
  nav.enableTimeLog(false);
  nav.enableRethrowNavExceptions(true);

  CAbstractNavigator::TNavigationParams np;
  np.target.target_coords = mrpt::math::TPose2D(1, 0, 0);
  nav.navigate(&np);
  EXPECT_ANY_THROW(nav.navigationStep());
}

TEST(RNavVariants, holonomic_method_can_be_switched_at_runtime)
{
  RNavFixture f;
  f.build(make_rnav_config(false, false, .0, true));

  mrpt::config::CConfigFileMemory cfg(make_rnav_config(false, false, .0, true));
  EXPECT_NO_THROW(f.nav->setHolonomicMethod("CHolonomicND", cfg));
  EXPECT_NO_THROW(f.nav->setHolonomicMethod("CHolonomicVFF", cfg));
  EXPECT_ANY_THROW(f.nav->setHolonomicMethod("NoSuchHolonomicMethod", cfg));
}

TEST(RNavVariants, config_file_roundtrip)
{
  RNavFixture f;
  f.build(make_rnav_config(true, true, 0.3, false));

  mrpt::config::CConfigFileMemory out;
  f.nav->saveConfigFile(out);

  EXPECT_TRUE(out.sectionExists("CAbstractPTGBasedReactive"));
  EXPECT_NEAR(out.read_double("CAbstractPTGBasedReactive", "speedfilter_tau", -1.0), 0.3, 1e-6);
  EXPECT_TRUE(out.read_bool("CAbstractPTGBasedReactive", "use_delays_model", false));
  EXPECT_TRUE(out.read_bool("CAbstractPTGBasedReactive", "evaluate_clearance", false));
  EXPECT_FALSE(out.read_bool("CAbstractPTGBasedReactive", "enable_obstacle_filtering", true));
}

TEST(RNavVariants, target_approach_slowdown_distance_is_forwarded_to_the_holo_method)
{
  RNavFixture f;
  f.build(make_rnav_config(false, false, .0, true));
  EXPECT_NO_THROW(f.nav->setTargetApproachSlowDownDistance(0.55));
}

TEST(RNavVariants, changing_the_robot_shape_after_initialization)
{
  RNavFixture f;
  f.build(make_rnav_config(false, false, .0, true));

  mrpt::math::CPolygon shape;
  shape.add_vertex(-0.2, 0.2);
  shape.add_vertex(0.2, 0.2);
  shape.add_vertex(0.2, -0.2);
  shape.add_vertex(-0.2, -0.2);
  EXPECT_NO_THROW(f.nav->changeRobotShape(shape));

  // A degenerate shape is rejected:
  mrpt::math::CPolygon tooFew;
  tooFew.add_vertex(0, 0);
  tooFew.add_vertex(1, 0);
  EXPECT_ANY_THROW(f.nav->changeRobotShape(tooFew));
}
