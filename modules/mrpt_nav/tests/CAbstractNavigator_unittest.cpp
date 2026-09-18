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

/** Unit tests for the navigation state machine implemented by
 *  CAbstractNavigator, and for the waypoint-sequence layer on top of it
 *  (CWaypointsNavigator), driven by a mock robot interface.
 */

#include <gtest/gtest.h>
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/kinematics/CVehicleVelCmd_DiffDriven.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/nav/reactive/CWaypointsNavigator.h>

#include <chrono>
#include <stdexcept>

using namespace mrpt::nav;

namespace
{
/** A robot interface that records every callback invoked by the navigator. */
struct MockRobotIF : public CRobot2NavInterface
{
  mrpt::math::TPose2D pose{0, 0, 0};
  bool poseIsAvailable = true;
  bool changeSpeedsSucceeds = true;
  double navTime = 0;

  int nChangeSpeeds = 0;
  int nChangeSpeedsNOP = 0;
  int nStop = 0;
  int nStartWatchdog = 0;
  int nStopWatchdog = 0;
  int nNavStart = 0;
  int nNavEnd = 0;
  int nNavEndDueToError = 0;
  int nWaypointReached = 0;
  int nWaypointSkipped = 0;
  int nNewWaypointTarget = 0;
  int nWaySeemsBlocked = 0;
  int nCannotGetCloser = 0;

  /** If set, getAlignCmd() returns it (i.e. this robot supports in-place
   * rotations). */
  bool supportsAlignCmd = false;

  std::optional<CurrentPoseAndSpeeds> getCurrentPoseAndSpeeds() override
  {
    if (!poseIsAvailable) return std::nullopt;
    CurrentPoseAndSpeeds ret;
    ret.pose = pose;
    ret.velGlobal = mrpt::math::TTwist2D(0, 0, 0);
    ret.timestamp = mrpt::Clock::now();
    ret.odometry = pose;
    return ret;
  }
  bool changeSpeeds(const mrpt::kinematics::CVehicleVelCmd&) override
  {
    nChangeSpeeds++;
    return changeSpeedsSucceeds;
  }
  bool changeSpeedsNOP() override
  {
    nChangeSpeedsNOP++;
    return true;
  }
  bool stop(StopType) override
  {
    nStop++;
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
  mrpt::kinematics::CVehicleVelCmd::Ptr getAlignCmd(const double) override
  {
    if (!supportsAlignCmd) return {};
    return std::make_shared<mrpt::kinematics::CVehicleVelCmd_DiffDriven>();
  }
  bool senseObstacles(mrpt::maps::CSimplePointsMap& obs, mrpt::system::TTimeStamp& ts) override
  {
    obs.clear();
    ts = mrpt::Clock::now();
    return true;
  }
  bool startWatchdog(float) override
  {
    nStartWatchdog++;
    return true;
  }
  bool stopWatchdog() override
  {
    nStopWatchdog++;
    return true;
  }
  void sendNavigationStartEvent() override { nNavStart++; }
  void sendNavigationEndEvent() override { nNavEnd++; }
  void sendNavigationEndDueToErrorEvent() override { nNavEndDueToError++; }
  void sendWaypointReachedEvent(int, bool reached_nSkipped) override
  {
    if (reached_nSkipped)
      nWaypointReached++;
    else
      nWaypointSkipped++;
  }
  void sendNewWaypointTargetEvent(int) override { nNewWaypointTarget++; }
  void sendWaySeemsBlockedEvent() override { nWaySeemsBlocked++; }
  void sendCannotGetCloserToBlockedTargetEvent() override { nCannotGetCloser++; }

  double getNavigationTime() override { return navTime; }
  void resetNavigationTimer() override { navTime = 0; }
};

/** Minimal concrete navigator: the robot is "teleported" to the target at a
 *  configurable speed so navigation actually converges. */
struct TestNavigator : public CWaypointsNavigator
{
  MockRobotIF& m_mock;
  bool reachable = true;
  bool throwOnStep = false;
  bool emergencyStopOnStep = false;
  bool targetIsColliding = false;
  int nPerformNavigationStep = 0;

  TestNavigator(MockRobotIF& r) : CWaypointsNavigator(r), m_mock(r)
  {
    this->setMinLoggingLevel(mrpt::system::LVL_ERROR);
    m_navProfiler.enable(false);
  }

  bool impl_waypoint_is_reachable(const mrpt::math::TPoint2D&) const override { return reachable; }
  using CWaypointsNavigator::waypoints_isAligning;
  void performNavigationStep() override
  {
    nPerformNavigationStep++;
    if (throwOnStep) throw std::runtime_error("synthetic navigation failure");
    if (emergencyStopOnStep) this->doEmergencyStop("synthetic emergency stop");
  }
  bool checkCollisionWithLatestObstacles(const mrpt::math::TPose2D&) const override
  {
    return targetIsColliding;
  }
  void loadConfigFile(const mrpt::config::CConfigFileBase& c) override
  {
    CWaypointsNavigator::loadConfigFile(c);
  }
  void saveConfigFile(mrpt::config::CConfigFileBase& c) const override
  {
    CWaypointsNavigator::saveConfigFile(c);
  }
  void initialize() override {}

  /** Advances the robot's navigation clock and runs one navigation cycle. */
  void step(double dt = 0.5)
  {
    m_mock.navTime += dt;
    this->navigationStep();
  }

  /** Move the robot straight towards the current target. */
  void teleportTowardsTarget(double step)
  {
    if (!m_navigationParams) return;
    const auto trg = mrpt::math::TPoint2D(m_navigationParams->target.target_coords);
    auto d = trg - mrpt::math::TPoint2D(m_mock.pose);
    const double n = d.norm();
    if (n < step)
      m_mock.pose = mrpt::math::TPose2D(trg.x, trg.y, m_mock.pose.phi);
    else
    {
      d *= step / n;
      m_mock.pose.x += d.x;
      m_mock.pose.y += d.y;
    }
  }
};

CAbstractNavigator::TNavigationParams make_target(double x, double y, double allowedDist = 0.5)
{
  CAbstractNavigator::TNavigationParams np;
  np.target.target_coords = mrpt::math::TPose2D(x, y, 0);
  np.target.targetAllowedDistance = static_cast<float>(allowedDist);
  return np;
}
}  // namespace

// ---------------------------------------------------------------------------
//  TargetInfo / TNavigationParams value semantics
// ---------------------------------------------------------------------------
TEST(CAbstractNavigator, target_info_defaults_and_text)
{
  CAbstractNavigator::TargetInfo ti;
  EXPECT_EQ(ti.target_frame_id, "map");
  EXPECT_FALSE(ti.targetIsRelative);
  EXPECT_FALSE(ti.targetIsIntermediaryWaypoint);

  const std::string s = ti.getAsText();
  EXPECT_NE(s.find("target_coords"), std::string::npos);
  EXPECT_NE(s.find("targetAllowedDistance"), std::string::npos);
  EXPECT_NE(s.find("targetIsRelative = NO"), std::string::npos);
}

TEST(CAbstractNavigator, target_info_equality)
{
  CAbstractNavigator::TargetInfo a, b;
  EXPECT_TRUE(a == b);
  EXPECT_FALSE(a != b);

  b.target_coords = mrpt::math::TPose2D(1, 2, 3);
  EXPECT_TRUE(a != b);

  b = a;
  b.targetAllowedDistance = 9.0f;
  EXPECT_TRUE(a != b);

  b = a;
  b.targetIsRelative = true;
  EXPECT_TRUE(a != b);

  b = a;
  b.target_frame_id = "odom";
  EXPECT_TRUE(a != b);

  b = a;
  b.targetIsIntermediaryWaypoint = true;
  EXPECT_TRUE(a != b);

  b = a;
  b.targetDesiredRelSpeed = 0.9;
  EXPECT_TRUE(a != b);
}

TEST(CAbstractNavigator, nav_params_equality_is_type_aware)
{
  CAbstractNavigator::TNavigationParams a, b;
  EXPECT_TRUE(a == b);

  b.target.target_coords = mrpt::math::TPose2D(1, 0, 0);
  EXPECT_FALSE(a == b);

  // Different dynamic types are never equal:
  CWaypointsNavigator::TNavigationParamsWaypoints c;
  EXPECT_FALSE(a == c);

  EXPECT_NE(a.getAsText().find("Single target"), std::string::npos);

  auto cloned = a.clone();
  ASSERT_TRUE(cloned);
  EXPECT_TRUE(a == *cloned);
}

TEST(CWaypointsNavigator, waypoints_nav_params_text_and_equality)
{
  CWaypointsNavigator::TNavigationParamsWaypoints a, b;
  EXPECT_TRUE(a == b);

  CAbstractNavigator::TargetInfo ti;
  ti.target_coords = mrpt::math::TPose2D(1, 2, 0);
  b.multiple_targets.push_back(ti);
  EXPECT_FALSE(a == b);
  EXPECT_NE(b.getAsText().find("multiple_targets"), std::string::npos);

  auto cloned = b.clone();
  ASSERT_TRUE(cloned);
  EXPECT_TRUE(b == *cloned);
}

// ---------------------------------------------------------------------------
//  State machine
// ---------------------------------------------------------------------------
TEST(CAbstractNavigator, state_machine_idle_navigating_suspended)
{
  MockRobotIF robot;
  TestNavigator nav(robot);

  EXPECT_EQ(nav.getCurrentState(), CAbstractNavigator::TState::IDLE);

  auto np = make_target(10, 0);
  nav.navigate(&np);
  EXPECT_EQ(nav.getCurrentState(), CAbstractNavigator::TState::NAVIGATING);

  nav.navigationStep();
  EXPECT_EQ(nav.getCurrentState(), CAbstractNavigator::TState::NAVIGATING);
  EXPECT_EQ(robot.nStartWatchdog, 1);
  EXPECT_EQ(robot.nNavStart, 1);
  EXPECT_GT(nav.nPerformNavigationStep, 0);

  nav.suspend();
  EXPECT_EQ(nav.getCurrentState(), CAbstractNavigator::TState::SUSPENDED);
  const int stopsAfterSuspend = robot.nStop;
  EXPECT_GT(stopsAfterSuspend, 0);

  // While suspended, no navigation step is run:
  const int nStepsBefore = nav.nPerformNavigationStep;
  nav.navigationStep();
  EXPECT_EQ(nav.nPerformNavigationStep, nStepsBefore);
  EXPECT_EQ(robot.nStopWatchdog, 1);

  nav.resume();
  EXPECT_EQ(nav.getCurrentState(), CAbstractNavigator::TState::NAVIGATING);

  nav.cancel();
  EXPECT_EQ(nav.getCurrentState(), CAbstractNavigator::TState::IDLE);
}

TEST(CAbstractNavigator, resume_and_suspend_are_noops_in_other_states)
{
  MockRobotIF robot;
  TestNavigator nav(robot);

  nav.resume();  // not suspended
  EXPECT_EQ(nav.getCurrentState(), CAbstractNavigator::TState::IDLE);

  nav.suspend();  // not navigating
  EXPECT_EQ(nav.getCurrentState(), CAbstractNavigator::TState::IDLE);

  nav.resetNavError();  // not in error
  EXPECT_EQ(nav.getCurrentState(), CAbstractNavigator::TState::IDLE);
}

TEST(CAbstractNavigator, reaching_the_target_ends_the_navigation)
{
  MockRobotIF robot;
  TestNavigator nav(robot);

  auto np = make_target(1.0, 0.0, 0.5);
  nav.navigate(&np);

  for (int i = 0; i < 20 && nav.getCurrentState() != CAbstractNavigator::TState::IDLE; i++)
  {
    nav.step();
    nav.teleportTowardsTarget(0.3);
  }

  EXPECT_EQ(nav.getCurrentState(), CAbstractNavigator::TState::IDLE);
  EXPECT_EQ(robot.nNavEnd, 1);
}

TEST(CAbstractNavigator, relative_targets_are_converted_to_absolute)
{
  MockRobotIF robot;
  robot.pose = mrpt::math::TPose2D(10, 20, .0);
  TestNavigator nav(robot);

  auto np = make_target(1.0, 2.0);
  np.target.targetIsRelative = true;
  nav.navigate(&np);

  nav.navigationStep();
  EXPECT_EQ(nav.getCurrentState(), CAbstractNavigator::TState::NAVIGATING);
}

TEST(CAbstractNavigator, navigate_rejects_null_or_out_of_range_params)
{
  MockRobotIF robot;
  TestNavigator nav(robot);

  EXPECT_ANY_THROW(nav.navigate(nullptr));

  auto np = make_target(1, 0);
  np.target.targetDesiredRelSpeed = 5.0;  // out of [0,1]
  EXPECT_ANY_THROW(nav.navigate(&np));
}

TEST(CAbstractNavigator, unavailable_robot_pose_raises_a_nav_error)
{
  MockRobotIF robot;
  TestNavigator nav(robot);

  auto np = make_target(10, 0);
  nav.navigate(&np);

  robot.poseIsAvailable = false;
  nav.navigationStep();

  EXPECT_EQ(nav.getCurrentState(), CAbstractNavigator::TState::NAV_ERROR);
  EXPECT_EQ(nav.getErrorReason().error_code, CAbstractNavigator::TErrorCode::EMERGENCY_STOP);
  EXPECT_FALSE(nav.getErrorReason().error_msg.empty());

  // Only resetNavError() gets us out of it:
  nav.resetNavError();
  EXPECT_EQ(nav.getCurrentState(), CAbstractNavigator::TState::IDLE);
  EXPECT_EQ(nav.getErrorReason().error_code, CAbstractNavigator::TErrorCode::NONE);
}

TEST(CAbstractNavigator, exceptions_in_the_nav_step_raise_a_nav_error)
{
  MockRobotIF robot;
  TestNavigator nav(robot);

  auto np = make_target(10, 0);
  nav.navigate(&np);
  nav.throwOnStep = true;
  nav.navigationStep();

  EXPECT_EQ(nav.getCurrentState(), CAbstractNavigator::TState::NAV_ERROR);
  EXPECT_EQ(nav.getErrorReason().error_code, CAbstractNavigator::TErrorCode::OTHER);
  EXPECT_NE(nav.getErrorReason().error_msg.find("synthetic navigation failure"), std::string::npos);

  // The next step reports the error to the robot interface and stops it:
  nav.navigationStep();
  EXPECT_EQ(robot.nNavEndDueToError, 1);
}

TEST(CAbstractNavigator, exceptions_can_be_rethrown_to_the_caller)
{
  MockRobotIF robot;
  TestNavigator nav(robot);

  EXPECT_FALSE(nav.isRethrowNavExceptionsEnabled());
  nav.enableRethrowNavExceptions(true);
  EXPECT_TRUE(nav.isRethrowNavExceptionsEnabled());

  auto np = make_target(10, 0);
  nav.navigate(&np);
  nav.throwOnStep = true;
  EXPECT_ANY_THROW(nav.navigationStep());
  EXPECT_EQ(nav.getCurrentState(), CAbstractNavigator::TState::NAV_ERROR);
}

TEST(CAbstractNavigator, emergency_stop_latches_the_error_state)
{
  MockRobotIF robot;
  TestNavigator nav(robot);

  auto np = make_target(10, 0);
  nav.navigate(&np);
  nav.emergencyStopOnStep = true;
  nav.navigationStep();

  EXPECT_EQ(nav.getCurrentState(), CAbstractNavigator::TState::NAV_ERROR);
  EXPECT_EQ(nav.getErrorReason().error_code, CAbstractNavigator::TErrorCode::EMERGENCY_STOP);
  EXPECT_NE(nav.getErrorReason().error_msg.find("synthetic emergency stop"), std::string::npos);
}

TEST(CAbstractNavigator, a_blocked_target_raises_the_corresponding_event)
{
  MockRobotIF robot;
  TestNavigator nav(robot);
  nav.params_abstract_navigator.dist_check_target_is_blocked = 5.0;
  nav.params_abstract_navigator.hysteresis_check_target_is_blocked = 2;

  auto np = make_target(1.0, 0.0, 0.1 /*never reached*/);
  nav.navigate(&np);
  nav.targetIsColliding = true;

  for (int i = 0; i < 4; i++)
  {
    nav.step();
  }
  EXPECT_GT(robot.nCannotGetCloser, 0);

  // Once the obstacle is gone the hysteresis counter is reset:
  const int n0 = robot.nCannotGetCloser;
  nav.targetIsColliding = false;
  for (int i = 0; i < 4; i++)
  {
    nav.step();
  }
  EXPECT_EQ(robot.nCannotGetCloser, n0);
}

TEST(CAbstractNavigator, not_approaching_the_target_times_out)
{
  MockRobotIF robot;
  TestNavigator nav(robot);
  nav.params_abstract_navigator.alarm_seems_not_approaching_target_timeout = 2.0;

  // The alarm compares wall-clock timestamps, so drive it from the simulated
  // clock: a plain "0 s timeout" would depend on the platform clock
  // resolution being fine enough for two consecutive now() calls to differ.
  const auto savedClock = mrpt::Clock::getActiveClock();
  mrpt::Clock::setSimulatedTime(mrpt::Clock::now());
  mrpt::Clock::setActiveClock(mrpt::Clock::Simulated);

  auto np = make_target(10.0, 0.0, 0.1);
  nav.navigate(&np);

  // The robot never moves, so the "not approaching" alarm must fire:
  for (int i = 0; i < 10 && nav.getCurrentState() == CAbstractNavigator::TState::NAVIGATING; i++)
  {
    nav.step();
    mrpt::Clock::setSimulatedTime(mrpt::Clock::now() + std::chrono::milliseconds(500));
  }
  mrpt::Clock::setActiveClock(savedClock);

  EXPECT_EQ(nav.getCurrentState(), CAbstractNavigator::TState::NAV_ERROR);
  EXPECT_EQ(nav.getErrorReason().error_code, CAbstractNavigator::TErrorCode::CANNOT_REACH_TARGET);
  EXPECT_GT(robot.nWaySeemsBlocked, 0);
}

TEST(CAbstractNavigator, end_of_nav_event_distance_is_configurable)
{
  MockRobotIF robot;
  TestNavigator nav(robot);
  // Send the "navigation end" event well before the target is reached:
  nav.params_abstract_navigator.dist_to_target_for_sending_event = 3.0;

  auto np = make_target(1.0, 0.0, 0.2);
  nav.navigate(&np);
  nav.navigationStep();

  EXPECT_EQ(robot.nNavEnd, 1);
}

TEST(CAbstractNavigator, config_file_roundtrip)
{
  MockRobotIF robot;
  TestNavigator nav(robot);

  nav.params_abstract_navigator.dist_to_target_for_sending_event = 1.25;
  nav.params_abstract_navigator.alarm_seems_not_approaching_target_timeout = 12.5;
  nav.params_abstract_navigator.dist_check_target_is_blocked = 3.5;
  nav.params_abstract_navigator.hysteresis_check_target_is_blocked = 7;
  nav.params_waypoints_navigator.max_distance_to_allow_skip_waypoint = 4.5;
  nav.params_waypoints_navigator.min_timesteps_confirm_skip_waypoints = 3;
  nav.params_waypoints_navigator.multitarget_look_ahead = 2;
  nav.params_waypoints_navigator.minimum_target_approach_per_step = 0.05;

  mrpt::config::CConfigFileMemory cfg;
  nav.saveConfigFile(cfg);

  MockRobotIF robot2;
  TestNavigator nav2(robot2);
  nav2.loadConfigFile(cfg);

  EXPECT_NEAR(nav2.params_abstract_navigator.dist_to_target_for_sending_event, 1.25, 1e-9);
  EXPECT_NEAR(
      nav2.params_abstract_navigator.alarm_seems_not_approaching_target_timeout, 12.5, 1e-9);
  EXPECT_NEAR(nav2.params_abstract_navigator.dist_check_target_is_blocked, 3.5, 1e-9);
  EXPECT_EQ(nav2.params_abstract_navigator.hysteresis_check_target_is_blocked, 7);
  EXPECT_NEAR(nav2.params_waypoints_navigator.max_distance_to_allow_skip_waypoint, 4.5, 1e-9);
  EXPECT_EQ(nav2.params_waypoints_navigator.min_timesteps_confirm_skip_waypoints, 3);
  EXPECT_EQ(nav2.params_waypoints_navigator.multitarget_look_ahead, 2);
  EXPECT_NEAR(nav2.params_waypoints_navigator.minimum_target_approach_per_step, 0.05, 1e-9);
}

TEST(CAbstractNavigator, frame_transformer_is_stored)
{
  MockRobotIF robot;
  TestNavigator nav(robot);
  EXPECT_TRUE(nav.getFrameTF().expired());

  auto tf = std::make_shared<mrpt::poses::FrameTransformer<2>>();
  nav.setFrameTF(tf);
  EXPECT_FALSE(nav.getFrameTF().expired());
}

// ---------------------------------------------------------------------------
//  Waypoint sequences
// ---------------------------------------------------------------------------
TEST(CWaypointsNavigator, navigates_through_a_whole_sequence)
{
  MockRobotIF robot;
  TestNavigator nav(robot);
  nav.reachable = false;  // disable the skip policy: follow them one by one

  TWaypointSequence seq;
  seq.waypoints.emplace_back(1.0, 0.0, 0.4);
  seq.waypoints.emplace_back(2.0, 0.0, 0.4);
  seq.waypoints.emplace_back(3.0, 0.0, 0.4);
  nav.navigateWaypoints(seq);

  for (int i = 0; i < 100 && !nav.getWaypointNavStatus().final_goal_reached; i++)
  {
    nav.step();
    nav.teleportTowardsTarget(0.25);
  }

  const auto st = nav.getWaypointNavStatus();
  EXPECT_TRUE(st.final_goal_reached);
  EXPECT_EQ(st.waypoint_index_current_goal, 2);
  for (const auto& wp : st.waypoints)
  {
    EXPECT_TRUE(wp.reached);
    EXPECT_FALSE(wp.skipped);
    EXPECT_NE(wp.timestamp_reach, INVALID_TIMESTAMP);
  }
  EXPECT_GE(robot.nWaypointReached, 3);
  EXPECT_GE(robot.nNewWaypointTarget, 3);
  EXPECT_EQ(robot.nNavEnd, 1);
}

TEST(CWaypointsNavigator, skips_reachable_intermediary_waypoints)
{
  MockRobotIF robot;
  TestNavigator nav(robot);
  nav.reachable = true;  // every waypoint looks directly reachable
  nav.params_waypoints_navigator.min_timesteps_confirm_skip_waypoints = 0;

  TWaypointSequence seq;
  seq.waypoints.emplace_back(1.0, 0.0, 0.4);
  seq.waypoints.emplace_back(2.0, 0.0, 0.4);
  seq.waypoints.emplace_back(3.0, 0.0, 0.4);
  nav.navigateWaypoints(seq);

  // The first cycle only activates waypoint #0; the skip policy runs from the
  // second one on:
  nav.step();
  nav.step();

  const auto st = nav.getWaypointNavStatus();
  // It jumped straight to the last waypoint, marking the others as skipped:
  EXPECT_EQ(st.waypoint_index_current_goal, 2);
  EXPECT_TRUE(st.waypoints[0].skipped);
  EXPECT_TRUE(st.waypoints[0].reached);
  EXPECT_TRUE(st.waypoints[1].skipped);
  EXPECT_FALSE(st.waypoints[2].reached);
  EXPECT_EQ(robot.nWaypointSkipped, 2);
}

TEST(CWaypointsNavigator, non_skippable_waypoints_are_honored)
{
  MockRobotIF robot;
  TestNavigator nav(robot);
  nav.reachable = true;
  nav.params_waypoints_navigator.min_timesteps_confirm_skip_waypoints = 0;

  TWaypointSequence seq;
  seq.waypoints.emplace_back(1.0, 0.0, 0.4, false /*allow_skip*/);
  seq.waypoints.emplace_back(2.0, 0.0, 0.4);
  nav.navigateWaypoints(seq);

  nav.navigationStep();

  const auto st = nav.getWaypointNavStatus();
  EXPECT_EQ(st.waypoint_index_current_goal, 0);
  EXPECT_FALSE(st.waypoints[0].skipped);
}

TEST(CWaypointsNavigator, far_away_waypoints_are_not_skipped_to)
{
  MockRobotIF robot;
  TestNavigator nav(robot);
  nav.reachable = true;
  nav.params_waypoints_navigator.min_timesteps_confirm_skip_waypoints = 0;
  nav.params_waypoints_navigator.max_distance_to_allow_skip_waypoint = 1.5;

  TWaypointSequence seq;
  seq.waypoints.emplace_back(1.0, 0.0, 0.4);
  seq.waypoints.emplace_back(50.0, 0.0, 0.4);  // beyond the look-ahead radius
  nav.navigateWaypoints(seq);

  nav.navigationStep();

  const auto st = nav.getWaypointNavStatus();
  EXPECT_EQ(st.waypoint_index_current_goal, 0);
}

TEST(CWaypointsNavigator, multitarget_look_ahead_forwards_extra_targets)
{
  MockRobotIF robot;
  TestNavigator nav(robot);
  nav.reachable = false;
  nav.params_waypoints_navigator.multitarget_look_ahead = 2;

  TWaypointSequence seq;
  seq.waypoints.emplace_back(1.0, 0.0, 0.4);
  seq.waypoints.emplace_back(2.0, 0.0, 0.4);
  seq.waypoints.emplace_back(3.0, 0.0, 0.4);
  nav.navigateWaypoints(seq);

  nav.navigationStep();
  EXPECT_EQ(nav.getCurrentState(), CAbstractNavigator::TState::NAVIGATING);
  EXPECT_EQ(robot.nNewWaypointTarget, 1);
}

TEST(CWaypointsNavigator, empty_waypoint_sequences_are_rejected)
{
  MockRobotIF robot;
  TestNavigator nav(robot);
  TWaypointSequence seq;
  EXPECT_ANY_THROW(nav.navigateWaypoints(seq));

  // ... and so are incompletely-filled waypoints:
  seq.waypoints.emplace_back();
  EXPECT_ANY_THROW(nav.navigateWaypoints(seq));
}

TEST(CWaypointsNavigator, cancel_clears_the_waypoint_status)
{
  MockRobotIF robot;
  TestNavigator nav(robot);

  TWaypointSequence seq;
  seq.waypoints.emplace_back(1.0, 0.0, 0.4);
  nav.navigateWaypoints(seq);
  ASSERT_EQ(nav.getWaypointNavStatus().waypoints.size(), 1U);

  nav.cancel();
  EXPECT_TRUE(nav.getWaypointNavStatus().waypoints.empty());
  EXPECT_EQ(nav.getWaypointNavStatus().timestamp_nav_started, INVALID_TIMESTAMP);
  EXPECT_EQ(nav.getCurrentState(), CAbstractNavigator::TState::IDLE);
}

TEST(CWaypointsNavigator, waypoints_emptied_mid_navigation_do_not_crash)
{
  MockRobotIF robot;
  TestNavigator nav(robot);

  TWaypointSequence seq;
  seq.waypoints.emplace_back(10.0, 0.0, 0.4);
  nav.navigateWaypoints(seq);
  nav.navigationStep();

  // A user of the access guard may legitimately empty the list:
  {
    auto guard = nav.getWaypointsAccessGuard();
    guard.waypoints().waypoints.clear();
  }

  EXPECT_NO_THROW(nav.navigationStep());
}

TEST(CWaypointsNavigator, waypoints_with_a_heading_align_before_being_reached)
{
  MockRobotIF robot;
  robot.supportsAlignCmd = true;
  TestNavigator nav(robot);
  nav.reachable = false;

  TWaypointSequence seq;
  seq.waypoints.emplace_back(1.0, 0.0, 0.4, true, mrpt::DEG2RAD(90.0));
  nav.navigateWaypoints(seq);

  // The alignment settling window is measured against the wall clock; freeze
  // a simulated one so the outcome does not depend on how fast the loop below
  // happens to run.
  const auto savedClock = mrpt::Clock::getActiveClock();
  mrpt::Clock::setSimulatedTime(mrpt::Clock::now());
  mrpt::Clock::setActiveClock(mrpt::Clock::Simulated);

  // Drive the robot right onto the waypoint, but with the wrong heading:
  for (int i = 0; i < 10; i++)
  {
    nav.step();
    nav.teleportTowardsTarget(0.5);
  }
  mrpt::Clock::setActiveClock(savedClock);

  // The robot is at the waypoint but not aligned: the navigator requests an
  // in-place rotation instead of declaring the waypoint reached.
  EXPECT_TRUE(nav.waypoints_isAligning());
  EXPECT_FALSE(nav.getWaypointNavStatus().final_goal_reached);
  EXPECT_GT(robot.nChangeSpeeds, 0);
}

TEST(CWaypointsNavigator, headed_waypoints_are_reached_when_alignment_is_unsupported)
{
  MockRobotIF robot;
  robot.supportsAlignCmd = false;  // this robot cannot rotate in place
  TestNavigator nav(robot);
  nav.reachable = false;

  TWaypointSequence seq;
  seq.waypoints.emplace_back(1.0, 0.0, 0.4, true, mrpt::DEG2RAD(90.0));
  nav.navigateWaypoints(seq);

  for (int i = 0; i < 30 && !nav.getWaypointNavStatus().final_goal_reached; i++)
  {
    nav.step();
    nav.teleportTowardsTarget(0.5);
  }
  EXPECT_TRUE(nav.getWaypointNavStatus().final_goal_reached);
}

TEST(CWaypointsNavigator, isRelativePointReachable_forwards_to_the_implementation)
{
  MockRobotIF robot;
  TestNavigator nav(robot);

  nav.reachable = true;
  EXPECT_TRUE(nav.isRelativePointReachable(mrpt::math::TPoint2D(1, 0)));
  nav.reachable = false;
  EXPECT_FALSE(nav.isRelativePointReachable(mrpt::math::TPoint2D(1, 0)));
}
