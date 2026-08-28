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

/** Coverage for the ready-made robot interfaces backed by the kinematic
 *  simulators, the pre-programmed velocity-sequence navigator, and the
 *  3D (multi-height) reactive navigation system.
 */

#include <gtest/gtest.h>
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/kinematics/CVehicleSimul_DiffDriven.h>
#include <mrpt/kinematics/CVehicleSimul_Holo.h>
#include <mrpt/nav/reactive/CNavigatorManualSequence.h>
#include <mrpt/nav/reactive/CReactiveNavigationSystem3D.h>
#include <mrpt/nav/reactive/CRobot2NavInterfaceForSimulator.h>

#include <string>

using namespace mrpt::nav;

// ---------------------------------------------------------------------------
//  CRobot2NavInterfaceForSimulator_{Holo,DiffDriven}
// ---------------------------------------------------------------------------
namespace
{
struct HoloIF : public CRobot2NavInterfaceForSimulator_Holo
{
  using CRobot2NavInterfaceForSimulator_Holo::CRobot2NavInterfaceForSimulator_Holo;
  bool senseObstacles(mrpt::maps::CSimplePointsMap& o, mrpt::system::TTimeStamp& ts) override
  {
    o.clear();
    ts = mrpt::Clock::now();
    return true;
  }
};

struct DiffIF : public CRobot2NavInterfaceForSimulator_DiffDriven
{
  using CRobot2NavInterfaceForSimulator_DiffDriven::CRobot2NavInterfaceForSimulator_DiffDriven;
  bool senseObstacles(mrpt::maps::CSimplePointsMap& o, mrpt::system::TTimeStamp& ts) override
  {
    o.clear();
    ts = mrpt::Clock::now();
    return true;
  }
};
}  // namespace

TEST(RobotIFForSimulator, holo_interface_drives_the_simulator)
{
  mrpt::kinematics::CVehicleSimul_Holo simul;
  HoloIF robot(simul);
  robot.setMinLoggingLevel(mrpt::system::LVL_ERROR);

  const auto ps = robot.getCurrentPoseAndSpeeds();
  ASSERT_TRUE(ps);
  EXPECT_NEAR(ps->pose.x, .0, 1e-12);

  mrpt::kinematics::CVehicleVelCmd_Holo cmd(1.0, .0, 0.3, .0);
  EXPECT_TRUE(robot.changeSpeeds(cmd));
  simul.simulateOneTimeStep(1.0);
  EXPECT_GT(simul.getCurrentGTPose().x, 0.1);

  // Normal and emergency stops differ only in the ramp time:
  auto stopCmd = robot.getStopCmd();
  auto emergCmd = robot.getEmergencyStopCmd();
  ASSERT_TRUE(stopCmd);
  ASSERT_TRUE(emergCmd);
  EXPECT_TRUE(stopCmd->isStopCmd());
  EXPECT_TRUE(emergCmd->isStopCmd());
  EXPECT_GT(stopCmd->getVelCmdElement(2), emergCmd->getVelCmdElement(2));

  EXPECT_TRUE(robot.stop(StopType::Normal));
  EXPECT_TRUE(robot.stop(StopType::Emergency));

  // Aligning: the commanded rotation speed follows the sign of the error:
  auto alignL = robot.getAlignCmd(+1.0);
  auto alignR = robot.getAlignCmd(-1.0);
  ASSERT_TRUE(alignL);
  ASSERT_TRUE(alignR);
  EXPECT_GT(alignL->getVelCmdElement(3), .0);
  EXPECT_LT(alignR->getVelCmdElement(3), .0);

  // The navigation clock follows simulated, not wall-clock, time:
  robot.resetNavigationTimer();
  EXPECT_NEAR(robot.getNavigationTime(), .0, 1e-9);
  simul.simulateOneTimeStep(0.5);
  EXPECT_NEAR(robot.getNavigationTime(), 0.5, 0.01);
}

TEST(RobotIFForSimulator, diffdriven_interface_drives_the_simulator)
{
  mrpt::kinematics::CVehicleSimul_DiffDriven simul;
  DiffIF robot(simul);
  robot.setMinLoggingLevel(mrpt::system::LVL_ERROR);

  const auto ps = robot.getCurrentPoseAndSpeeds();
  ASSERT_TRUE(ps);

  mrpt::kinematics::CVehicleVelCmd_DiffDriven cmd;
  cmd.lin_vel = 1.0;
  cmd.ang_vel = .0;
  EXPECT_TRUE(robot.changeSpeeds(cmd));
  simul.simulateOneTimeStep(1.0);
  EXPECT_GT(simul.getCurrentGTPose().x, 0.5);

  auto stopCmd = robot.getStopCmd();
  ASSERT_TRUE(stopCmd);
  EXPECT_TRUE(stopCmd->isStopCmd());
  auto emergCmd = robot.getEmergencyStopCmd();
  ASSERT_TRUE(emergCmd);
  EXPECT_TRUE(emergCmd->isStopCmd());

  // The stop command is forwarded to the simulator, which ramps down to zero:
  EXPECT_TRUE(robot.stop(StopType::Normal));
  simul.simulateOneTimeStep(0.5);
  EXPECT_NEAR(simul.getV(), .0, 1e-9);

  robot.resetNavigationTimer();
  EXPECT_NEAR(robot.getNavigationTime(), .0, 1e-9);
  simul.simulateOneTimeStep(0.25);
  EXPECT_NEAR(robot.getNavigationTime(), 0.25, 0.01);
}

// ---------------------------------------------------------------------------
//  CNavigatorManualSequence
// ---------------------------------------------------------------------------
namespace
{
struct RecordingRobotIF : public CRobot2NavInterfaceForSimulator_DiffDriven
{
  int nChangeSpeeds = 0;
  int nStop = 0;
  bool changeSpeedsSucceeds = true;
  double navTime = 0;

  using CRobot2NavInterfaceForSimulator_DiffDriven::CRobot2NavInterfaceForSimulator_DiffDriven;

  bool senseObstacles(mrpt::maps::CSimplePointsMap& o, mrpt::system::TTimeStamp& ts) override
  {
    o.clear();
    ts = mrpt::Clock::now();
    return true;
  }
  bool changeSpeeds(const mrpt::kinematics::CVehicleVelCmd& c) override
  {
    nChangeSpeeds++;
    if (!changeSpeedsSucceeds) return false;
    return CRobot2NavInterfaceForSimulator_DiffDriven::changeSpeeds(c);
  }
  bool stop(StopType t) override
  {
    nStop++;
    return CRobot2NavInterfaceForSimulator_DiffDriven::stop(t);
  }
  double getNavigationTime() override { return navTime; }
  void resetNavigationTimer() override { navTime = 0; }
};

std::string manual_sequence_cfg()
{
  return "[CNavigatorManualSequence]\n"
         // t  lin_vel  ang_vel   (2 components => differential-driven)
         "cmd1 = 0.0 0.5 0.0\n"
         "cmd2 = 1.0 0.0 0.3\n";
}
}  // namespace

TEST(CNavigatorManualSequence, executes_the_programmed_sequence_in_order)
{
  mrpt::kinematics::CVehicleSimul_DiffDriven simul;
  RecordingRobotIF robot(simul);
  robot.setMinLoggingLevel(mrpt::system::LVL_ERROR);

  CNavigatorManualSequence nav(robot);
  nav.setMinLoggingLevel(mrpt::system::LVL_ERROR);

  mrpt::config::CConfigFileMemory cfg(manual_sequence_cfg());
  nav.loadConfigFile(cfg);
  ASSERT_EQ(nav.programmed_orders.size(), 2U);

  nav.initialize();

  // t=0: the first command fires:
  robot.navTime = 0.0;
  nav.navigationStep();
  EXPECT_EQ(robot.nChangeSpeeds, 1);
  EXPECT_EQ(nav.programmed_orders.size(), 1U);

  // still before t=1: nothing new:
  robot.navTime = 0.5;
  nav.navigationStep();
  EXPECT_EQ(robot.nChangeSpeeds, 1);

  // t>=1: the second command fires and the queue empties:
  robot.navTime = 1.5;
  nav.navigationStep();
  EXPECT_EQ(robot.nChangeSpeeds, 2);
  EXPECT_TRUE(nav.programmed_orders.empty());

  // Once empty, further steps are no-ops:
  nav.navigationStep();
  EXPECT_EQ(robot.nChangeSpeeds, 2);
}

TEST(CNavigatorManualSequence, a_rejected_command_triggers_an_emergency_stop)
{
  mrpt::kinematics::CVehicleSimul_DiffDriven simul;
  RecordingRobotIF robot(simul);
  robot.setMinLoggingLevel(mrpt::system::LVL_ERROR);
  robot.changeSpeedsSucceeds = false;

  CNavigatorManualSequence nav(robot);
  nav.setMinLoggingLevel(mrpt::system::LVL_ERROR);

  mrpt::config::CConfigFileMemory cfg(manual_sequence_cfg());
  nav.loadConfigFile(cfg);
  nav.initialize();

  robot.navTime = 0.0;
  nav.navigationStep();
  EXPECT_GT(robot.nStop, 0);
  // The failed command is kept at the head of the queue:
  EXPECT_EQ(nav.programmed_orders.size(), 2U);
}

TEST(CNavigatorManualSequence, holonomic_commands_are_recognized_by_arity)
{
  mrpt::kinematics::CVehicleSimul_DiffDriven simul;
  RecordingRobotIF robot(simul);
  CNavigatorManualSequence nav(robot);
  nav.setMinLoggingLevel(mrpt::system::LVL_ERROR);

  mrpt::config::CConfigFileMemory cfg(
      std::string{"[CNavigatorManualSequence]\n"
                  // 4 components => holonomic
                  "cmd1 = 0.0 1.0 0.2 0.5 0.1\n"});
  nav.loadConfigFile(cfg);
  ASSERT_EQ(nav.programmed_orders.size(), 1U);
  EXPECT_EQ(nav.programmed_orders.begin()->second.cmd_vel->getVelCmdLength(), 4U);
}

TEST(CNavigatorManualSequence, malformed_sequences_are_rejected)
{
  mrpt::kinematics::CVehicleSimul_DiffDriven simul;
  RecordingRobotIF robot(simul);
  CNavigatorManualSequence nav(robot);
  nav.setMinLoggingLevel(mrpt::system::LVL_ERROR);

  {  // too few tokens
    mrpt::config::CConfigFileMemory cfg(
        std::string{"[CNavigatorManualSequence]\n"
                    "cmd1 = 0.0 0.5\n"});
    EXPECT_ANY_THROW(nav.loadConfigFile(cfg));
  }
  {  // 3 velocity components: neither 2 nor 4
    mrpt::config::CConfigFileMemory cfg(
        std::string{"[CNavigatorManualSequence]\n"
                    "cmd1 = 0.0 0.5 0.1 0.2\n"});
    EXPECT_ANY_THROW(nav.loadConfigFile(cfg));
  }
}

TEST(CNavigatorManualSequence, initialize_requires_a_non_empty_sequence)
{
  mrpt::kinematics::CVehicleSimul_DiffDriven simul;
  RecordingRobotIF robot(simul);
  CNavigatorManualSequence nav(robot);
  nav.setMinLoggingLevel(mrpt::system::LVL_ERROR);
  EXPECT_ANY_THROW(nav.initialize());
}

TEST(CNavigatorManualSequence, saveConfigFile_is_a_no_op)
{
  mrpt::kinematics::CVehicleSimul_DiffDriven simul;
  RecordingRobotIF robot(simul);
  CNavigatorManualSequence nav(robot);

  mrpt::config::CConfigFileMemory cfg;
  EXPECT_NO_THROW(nav.saveConfigFile(cfg));
}

// ---------------------------------------------------------------------------
//  CReactiveNavigationSystem3D
// ---------------------------------------------------------------------------
namespace
{
/** Same shape as the 2D config used in rnav_variants_unittest.cpp, plus the
 *  multi-height-level robot description this class requires. */
std::string rnav3d_config()
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
  s += "speedfilter_tau = 0.0\n";
  s += "secure_distance_start = 0.05\n";
  s += "secure_distance_end = 0.15\n";
  s += "use_delays_model = false\n";
  s += "max_distance_predicted_actual_path = 0.15\n";
  s += "min_normalized_free_space_for_ptg_continuation = 0.2\n";
  s += "enable_obstacle_filtering = true\n";
  s += "evaluate_clearance = false\n";

  s += "[CPointCloudFilterByDistance]\n";
  s += "min_dist = 0.1\n";
  s += "angle_tolerance = 5.0\n";
  s += "too_old_seconds = 1.0\n";
  s += "previous_keyframes = 1\n";
  s += "max_deletion_ratio = 0.4\n";

  s += "[CHolonomicVFF]\n";
  s += "TARGET_SLOW_APPROACHING_DISTANCE = 0.10\n";
  s += "TARGET_ATTRACTIVE_FORCE = 20.0\n";

  s += "[CMultiObjectiveMotionOptimizerBase]\n";
  s += "score1_name = collision_free_distance_score\n";
  s += "score1_formula = collision_free_distance\n";
  s += "score2_name = euclidean_nearness\n";
  s += "score2_formula = 1/(1+dist_eucl_min^2)\n";
  s += "scores_to_normalize = \n";

  s += "[CMultiObjMotionOpt_Scalarization]\n";
  s += "scalar_score_formula = 0.5*collision_free_distance_score + 8.0*euclidean_nearness\n";

  // Two vertical sections of the robot body:
  s += "[CReactiveNavigationSystem3D]\n";
  s += "HEIGHT_LEVELS = 2\n";
  s += "LEVEL1_HEIGHT = 0.4\n";
  s += "LEVEL1_VECTORX = -0.2 0.2 0.2 -0.2\n";
  s += "LEVEL1_VECTORY = 0.2 0.2 -0.2 -0.2\n";
  s += "LEVEL2_HEIGHT = 0.6\n";
  s += "LEVEL2_VECTORX = -0.15 0.15 0.15 -0.15\n";
  s += "LEVEL2_VECTORY = 0.15 0.15 -0.15 -0.15\n";
  s += "PTG_COUNT = 1\n";
  s += "PTG1_Type = CPTG_DiffDrive_C\n";
  s += "PTG1_resolution = 0.10\n";
  s += "PTG1_refDistance = 4.0\n";
  s += "PTG1_num_paths = 31\n";
  s += "PTG1_v_max_mps = 1.0\n";
  s += "PTG1_w_max_dps = 60\n";
  s += "PTG1_K = 1.0\n";
  s += "PTG1_score_priority = 1.0\n";
  return s;
}
}  // namespace

TEST(CReactiveNavigationSystem3D, in_memory_config_run)
{
  mrpt::kinematics::CVehicleSimul_DiffDriven simul;
  DiffIF robot(simul);
  robot.setMinLoggingLevel(mrpt::system::LVL_ERROR);

  auto nav = std::make_unique<CReactiveNavigationSystem3D>(robot, false /*no console output*/);
  nav->setMinLoggingLevel(mrpt::system::LVL_ERROR);
  nav->enableTimeLog(false);

  mrpt::config::CConfigFileMemory cfg(rnav3d_config());
  nav->loadConfigFile(cfg);
  nav->initialize();

  EXPECT_EQ(nav->getPTG_count(), 1U);
  ASSERT_NE(nav->getPTG(0), nullptr);
  const auto* constNav = nav.get();
  EXPECT_NE(constNav->getPTG(0), nullptr);

  // The parameters survive a save/load round-trip:
  mrpt::config::CConfigFileMemory out;
  nav->saveConfigFile(out);
  EXPECT_TRUE(out.sectionExists("CAbstractPTGBasedReactive"));

  CAbstractNavigator::TNavigationParams np;
  np.target.target_coords = mrpt::math::TPose2D(2.0, .0, .0);
  np.target.targetAllowedDistance = 0.4f;
  nav->navigate(&np);

  const auto savedClock = mrpt::Clock::getActiveClock();
  mrpt::Clock::setSimulatedTime(mrpt::Clock::now());
  mrpt::Clock::setActiveClock(mrpt::Clock::Simulated);

  const auto dt = std::chrono::milliseconds(200);
  for (unsigned int i = 0; i < 60; i++)
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

  EXPECT_LT(
      (mrpt::math::TPoint2D(simul.getCurrentGTPose()) - mrpt::math::TPoint2D(2.0, .0)).norm(), 0.6);
}

TEST(CReactiveNavigationSystem3D, robot_shape_can_be_redefined)
{
  mrpt::kinematics::CVehicleSimul_DiffDriven simul;
  DiffIF robot(simul);
  CReactiveNavigationSystem3D nav(robot, false);
  nav.setMinLoggingLevel(mrpt::system::LVL_ERROR);
  nav.enableTimeLog(false);

  mrpt::config::CConfigFileMemory cfg(rnav3d_config());
  nav.loadConfigFile(cfg);
  nav.initialize();

  TRobotShape shape;
  shape.resize(2);
  for (size_t lvl = 0; lvl < 2; lvl++)
  {
    auto& poly = shape.polygon(lvl);
    poly.add_vertex(-0.25, 0.25);
    poly.add_vertex(0.25, 0.25);
    poly.add_vertex(0.25, -0.25);
    poly.add_vertex(-0.25, -0.25);
    shape.setHeight(lvl, 0.4 + 0.2 * lvl);
    shape.setRadius(lvl, 0.35);
  }
  EXPECT_NO_THROW(nav.changeRobotShape(shape));
  EXPECT_EQ(shape.getHeights().size(), 2U);
  EXPECT_NEAR(shape.getRadius(0), 0.35, 1e-9);
  EXPECT_NEAR(shape.getHeight(1), 0.6, 1e-9);

  // A degenerate level polygon is rejected:
  TRobotShape bad;
  bad.resize(1);
  bad.polygon(0).add_vertex(0, 0);
  bad.polygon(0).add_vertex(1, 0);
  EXPECT_ANY_THROW(nav.changeRobotShape(bad));
}
