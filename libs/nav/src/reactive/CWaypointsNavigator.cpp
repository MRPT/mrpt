/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2026, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "nav-precomp.h"  // Precomp header
//
#include <mrpt/core/lock_helper.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/nav/reactive/CWaypointsNavigator.h>
#include <mrpt/poses/CPose2D.h>

using namespace mrpt::nav;
using namespace std;

std::string CWaypointsNavigator::TNavigationParamsWaypoints::getAsText() const
{
  std::string s = TNavigationParams::getAsText();
  if (!multiple_targets.empty())
  {
    s += "multiple_targets:\n";
    int i = 0;
    for (const auto& e : multiple_targets)
    {
      s += mrpt::format("target[%i]:\n", i++);
      s += e.getAsText();
    }
  }
  return s;
}

bool CWaypointsNavigator::TNavigationParamsWaypoints::isEqual(
    const CAbstractNavigator::TNavigationParamsBase& rhs) const
{
  auto o = dynamic_cast<const CWaypointsNavigator::TNavigationParamsWaypoints*>(&rhs);
  return o != nullptr && CAbstractNavigator::TNavigationParams::isEqual(rhs) &&
         multiple_targets == o->multiple_targets;
}

CWaypointsNavigator::CWaypointsNavigator(CRobot2NavInterface& robot_if) :
    CAbstractNavigator(robot_if)
{
  m_last_alignment_cmd = mrpt::Clock::now();
}

CWaypointsNavigator::~CWaypointsNavigator() = default;
void CWaypointsNavigator::onNavigateCommandReceived()
{
  CAbstractNavigator::onNavigateCommandReceived();

  std::lock_guard<std::recursive_mutex> csl(m_nav_waypoints_cs);

  // This initializes all fields to initial values:
  m_waypoint_nav_status = {};
}

void CWaypointsNavigator::navigateWaypoints(const TWaypointSequence& nav_request)
{
  MRPT_START

  this->onNavigateCommandReceived();  // reset waypoint status to {}

  std::lock_guard<std::recursive_mutex> csl(m_nav_waypoints_cs);

  const size_t N = nav_request.waypoints.size();
  ASSERTMSG_(N > 0, "List of waypoints is empty!");

  m_waypoint_nav_status.waypoints.resize(N);
  // Copy waypoints fields data, leave status fields to defaults:
  for (size_t i = 0; i < N; i++)
  {
    ASSERT_(nav_request.waypoints[i].isValid());
    m_waypoint_nav_status.waypoints[i] = nav_request.waypoints[i];
  }
  m_waypoint_nav_status.timestamp_nav_started = mrpt::Clock::now();

  m_waypoint_nav_status.waypoint_index_current_goal = -1;  // Not started yet.

  // The main loop navigationStep() will iterate over waypoints and send them
  // to navigate()
  MRPT_END
}

void CWaypointsNavigator::getWaypointNavStatus(TWaypointStatusSequence& out_nav_status) const
{
  // Make sure the data structure is not under modification:
  auto lck = mrpt::lockHelper(m_nav_waypoints_cs);

  out_nav_status = m_waypoint_nav_status;
}

/** \callergraph */
void CWaypointsNavigator::cancel()
{
  {
    std::lock_guard<std::recursive_mutex> csl(m_nav_waypoints_cs);
    m_waypoint_nav_status = TWaypointStatusSequence();
  }
  CAbstractNavigator::cancel();
}

/** \callergraph */
void CWaypointsNavigator::waypoints_navigationStep()
{
  MRPT_START

  using mrpt::square;

  // shortcut to save typing
  TWaypointStatusSequence& wps = m_waypoint_nav_status;

  // --------------------------------------
  //     Waypoint navigation algorithm
  // --------------------------------------
  wps.is_aligning = false;  // the robot is aligning into a waypoint with a desired heading

  mrpt::system::CTimeLoggerEntry tle(m_timlog_delays, "CWaypointsNavigator::navigationStep()");
  auto lck = mrpt::lockHelper(m_nav_waypoints_cs);

  if (wps.waypoints.empty() || wps.final_goal_reached)
  {
    // No nav request is pending or it was canceled
  }
  else
  {
    // Get current robot pose:
    CAbstractNavigator::updateCurrentPoseAndSpeeds();

    internal_select_next_waypoint();
  }

  // Note: navigationStep() is called *after* this waypoints part,
  // in order to get end-of-navigation events *after* waypoints-related events.

  wps.was_aligning = wps.is_aligning;  // Let the next timestep know about this

  MRPT_END
}

void CWaypointsNavigator::internal_select_next_waypoint()
{
  TWaypointStatusSequence& wps = m_waypoint_nav_status;

  // -------------------------------------------------------------------------
  // 1) default policy: go thru WPs one by one
  // -------------------------------------------------------------------------
  const int prev_wp_index = wps.waypoint_index_current_goal;

  wps.robot_move_seg.point1.x = m_curPoseVel.pose.x;
  wps.robot_move_seg.point1.y = m_curPoseVel.pose.y;
  if (wps.last_robot_pose.x == TWaypoint::INVALID_NUM)
  {
    wps.robot_move_seg.point2 = wps.robot_move_seg.point1;
  }
  else
  {
    wps.robot_move_seg.point2.x = wps.last_robot_pose.x;
    wps.robot_move_seg.point2.y = wps.last_robot_pose.y;
  }
  wps.last_robot_pose = m_curPoseVel.pose;  // save for next iters

  decltype(m_pending_events) new_events;

  internal_select_next_waypoint_default_policy(new_events);

  // -------------------------------------------------------------------------
  // 2) More advanced policy: if available, use children class methods
  // to decide which is the best candidate for the next waypoint, if we can
  // skip current one:
  // -------------------------------------------------------------------------
  internal_select_next_waypoint_skip_policy(new_events);

  // Insert at the beginning, for these events to be dispatched
  // *before* any "end of nav" event:
  m_pending_events.insert(m_pending_events.begin(), new_events.begin(), new_events.end());

  // Still not started and no better guess?
  if (wps.waypoint_index_current_goal < 0)
  {
    // Start with the first waypoint:
    wps.waypoint_index_current_goal = 0;
  }

  // -------------------------------------------------------------------------
  // 3) Should I request a new (single target) navigation command?
  //    Only if the temporary goal changed:
  // -------------------------------------------------------------------------
  internal_send_new_nav_cmd(prev_wp_index);
}

// -------------------------------------------------------------------------
// default policy: go thru WPs one by one
// -------------------------------------------------------------------------
void CWaypointsNavigator::internal_select_next_waypoint_default_policy(
    std::list<std::function<void()>>& new_events)
{
  TWaypointStatusSequence& wps = m_waypoint_nav_status;

  if (wps.waypoint_index_current_goal < 0) return;  // no active navigation

  // Check if waypoint (WP) was reached:
  auto& wp = wps.waypoints[wps.waypoint_index_current_goal];
  const double dist2target = wps.robot_move_seg.distance(wp.target);

  const double prev_dist2target = wps.prevDist2target;
  wps.prevDist2target = dist2target;

  if (dist2target > wp.allowed_distance && !wps.was_aligning /* we were already aligning at a WP */)
    return;  // no need to check, we are not close enough.

  if (!wps.was_aligning && wps.prevDist2target > 0)
  {
    // We are approaching a WP, within |wp.allowed_distance| radius.
    // As long as we are getting closer and closer, let it keep going:
    if (dist2target <
        prev_dist2target - params_waypoints_navigator.minimum_target_approach_per_step)
    {  // ok, we are getting closer, do nothing:
      return;
    }
    // Continue and accept the WP as reached (or perform the alignment there)
  }

  bool consider_wp_reached = false;

  if (!wp.target_heading.has_value())
  {
    // We reached a WP without any desired heading, so we are done:
    consider_wp_reached = true;
  }
  else
  {
    // Handle pure-rotation robot interface to honor target_heading
    const double ang_err =
        mrpt::math::angDistance(m_curPoseVel.pose.phi, wp.target_heading.value());
    const double tim_since_last_align =
        mrpt::system::timeDifference(m_last_alignment_cmd, mrpt::Clock::now());
    const double ALIGN_WAIT_TIME = 1.5;  // seconds

    if (std::abs(ang_err) <= params_waypoints_navigator.waypoint_angle_tolerance &&
        /* give some time for the alignment (if supported in this robot) to finish */
        tim_since_last_align > ALIGN_WAIT_TIME)
    {
      consider_wp_reached = true;
    }
    else
    {
      wps.is_aligning = true;

      if (!wps.was_aligning)
      {
        // 1st time we are aligning:
        // Send vel_cmd to the robot:
        mrpt::kinematics::CVehicleVelCmd::Ptr align_cmd = m_robot.getAlignCmd(ang_err);

        MRPT_LOG_INFO_FMT(
            "[CWaypointsNavigator::navigationStep] Trying to align to heading: %.02f deg. Relative "
            "heading: %.02f deg. With motion cmd: %s",
            mrpt::RAD2DEG(*wp.target_heading), mrpt::RAD2DEG(ang_err),
            align_cmd ? align_cmd->asString().c_str()
                      : "nullptr (operation not supported by this robot)");

        // In any case, do a "stop"
        this->stop(false /*not emergency*/);

        if (align_cmd)
          this->changeSpeeds(*align_cmd);
        else
        {
          // this robot does not support "in place" alignment
          consider_wp_reached = true;
        }
      }
      else
      {
        MRPT_LOG_THROTTLE_INFO_FMT(
            0.5,
            "[CWaypointsNavigator::navigationStep] Waiting for the robot to get aligned: "
            "current_heading=%.02f deg target_heading=%.02f deg",
            mrpt::RAD2DEG(m_curPoseVel.pose.phi), mrpt::RAD2DEG(*wp.target_heading));
      }
    }
  }

  if (consider_wp_reached)
  {
    MRPT_LOG_DEBUG_STREAM(
        "[CWaypointsNavigator::navigationStep] Waypoint "
        << (wps.waypoint_index_current_goal + 1) << "/" << wps.waypoints.size()
        << " reached."
           " segment-to-target dist: "
        << dist2target << ", allowed_dist: " << wp.allowed_distance);

    wps.is_aligning = false;

    wp.reached = true;
    wp.skipped = false;
    wp.timestamp_reach = mrpt::Clock::now();

    new_events.emplace_back(std::bind(
        &CRobot2NavInterface::sendWaypointReachedEvent, std::ref(m_robot),
        wps.waypoint_index_current_goal, true /*reason: really reached*/));

    // Was this the final goal??
    if (wps.waypoint_index_current_goal < int(wps.waypoints.size() - 1))
    {
      wps.waypoint_index_current_goal++;
    }
    else
    {
      wps.final_goal_reached = true;
      // Make sure the end-navigation event is issued,
      // navigation state switches to IDLE, etc:
      this->performNavigationStepNavigating(false);
    }
  }
}

void CWaypointsNavigator::internal_select_next_waypoint_skip_policy(
    std::list<std::function<void(void)>>& new_events)
{
  TWaypointStatusSequence& wps = m_waypoint_nav_status;

  if (!wps.final_goal_reached && wps.waypoint_index_current_goal >= 0 &&
      wps.waypoints[wps.waypoint_index_current_goal].allow_skip)
  {
    const mrpt::poses::CPose2D robot_pose(m_curPoseVel.pose);
    int most_advanced_wp = wps.waypoint_index_current_goal;
    const int most_advanced_wp_at_begin = most_advanced_wp;

    for (int idx = wps.waypoint_index_current_goal; idx < (int)wps.waypoints.size(); idx++)
    {
      if (idx < 0) continue;
      if (wps.waypoints[idx].reached) continue;

      // Is it reachable?
      mrpt::math::TPoint2D wp_local_wrt_robot;
      robot_pose.inverseComposePoint(wps.waypoints[idx].target, wp_local_wrt_robot);

      if (params_waypoints_navigator.max_distance_to_allow_skip_waypoint > 0 &&
          wp_local_wrt_robot.norm() >
              params_waypoints_navigator.max_distance_to_allow_skip_waypoint)
        continue;  // Skip this one, it is too far away

      const bool is_reachable = this->impl_waypoint_is_reachable(wp_local_wrt_robot);

      if (is_reachable)
      {
        // Robustness filter: only skip to a future waypoint if
        // it is seen as "reachable" during
        // a given number of timesteps:
        if (++wps.waypoints[idx].counter_seen_reachable >
            params_waypoints_navigator.min_timesteps_confirm_skip_waypoints)
        {
          most_advanced_wp = idx;
        }
      }

      // Is allowed to skip it?
      if (!wps.waypoints[idx].allow_skip)
        break;  // Do not keep trying, since we are now allowed
                // to skip this one.
    }

    if (most_advanced_wp >= 0)
    {
      wps.waypoint_index_current_goal = most_advanced_wp;
      for (int k = most_advanced_wp_at_begin; k < most_advanced_wp; k++)
      {
        auto& wp = wps.waypoints[k];
        wp.reached = true;
        wp.skipped = true;
        wp.timestamp_reach = mrpt::Clock::now();

        new_events.emplace_back(std::bind(
            &CRobot2NavInterface::sendWaypointReachedEvent, std::ref(m_robot), k,
            false /*reason: skipped*/));
      }
    }
  }
}

void CWaypointsNavigator::internal_send_new_nav_cmd(const int prev_wp_index)
{
  TWaypointStatusSequence& wps = m_waypoint_nav_status;

  // Should I request a new (single target) navigation command?
  // Only if the temporary goal changed:
  if (wps.waypoint_index_current_goal < 0 || prev_wp_index == wps.waypoint_index_current_goal)
    return;  // no changes

  ASSERT_(wps.waypoint_index_current_goal < int(wps.waypoints.size()));
  ASSERT_(params_waypoints_navigator.multitarget_look_ahead >= 0);

  // Notify we have a new "current waypoint"
  // Push back so it's dispatched *after* the wp reached events:
  m_pending_events.emplace_back(std::bind(
      &CRobot2NavInterface::sendNewWaypointTargetEvent, std::ref(m_robot),
      wps.waypoint_index_current_goal));

  // Send the current targets + "multitarget_look_ahead"
  // additional ones to help the local planner.
  CWaypointsNavigator::TNavigationParamsWaypoints nav_cmd;

  // Check skippable flag while traversing from current wp forward
  // "multitarget_look_ahead" steps:
  int wp_last_idx = wps.waypoint_index_current_goal;
  for (int nstep = 0; wp_last_idx < int(wps.waypoints.size()) - 1 &&
                      nstep < params_waypoints_navigator.multitarget_look_ahead;
       ++nstep)
  {
    if (!m_waypoint_nav_status.waypoints[wp_last_idx].allow_skip) break;
    wp_last_idx++;
  }

  for (int wp_idx = wps.waypoint_index_current_goal; wp_idx <= wp_last_idx; wp_idx++)
  {
    TWaypointStatus& wp = wps.waypoints[wp_idx];
    const bool is_final_wp = ((wp_idx + 1) == int(wps.waypoints.size()));

    CAbstractNavigator::TargetInfo ti;

    ti.target_coords.x = wp.target.x;
    ti.target_coords.y = wp.target.y;
    ti.target_coords.phi = (wp.target_heading.has_value() ? *wp.target_heading : .0);
    ti.target_frame_id = wp.target_frame_id;
    ti.targetAllowedDistance = wp.allowed_distance;
    ti.targetIsRelative = false;
    ti.targetIsIntermediaryWaypoint = !is_final_wp;
    ti.targetDesiredRelSpeed = wp.speed_ratio;

    // For backwards compat. with single-target code, write
    // single target info too for the first, next, waypoint:
    if (wp_idx == wps.waypoint_index_current_goal)
    {
      nav_cmd.target = ti;
    }
    // Append to list of targets:
    nav_cmd.multiple_targets.emplace_back(ti);
  }
  this->processNavigateCommand(&nav_cmd);

  MRPT_LOG_DEBUG_STREAM(
      "[CWaypointsNavigator::navigationStep] Active waypoint "
      "changed. Current status:\n"
      << this->getWaypointNavStatus().getAsText());
}

bool CWaypointsNavigator::waypoints_isAligning() const
{
  auto lck = mrpt::lockHelper(m_nav_waypoints_cs);
  return m_waypoint_nav_status.is_aligning;
}

void CWaypointsNavigator::navigationStep()
{
  MRPT_START

  m_waypoint_nav_status.is_aligning = false;

  mrpt::system::CTimeLoggerEntry tle(m_navProfiler, "CWaypointsNavigator::navigationStep()");

  if (m_navigationState != SUSPENDED)
  {
    waypoints_navigationStep();
  }

  // Call base navigation step to execute one-single waypoint navigation, as
  // usual:
  if (!m_waypoint_nav_status.is_aligning)
  {
    CAbstractNavigator::navigationStep();  // This internally locks "m_nav_cs"
  }
  else
  {
    // otherwise, at least, process pending events:
    CAbstractNavigator::dispatchPendingNavEvents();
  }

  MRPT_END
}

/** \callergraph */
void CWaypointsNavigator::onStartNewNavigation() {}
/** \callergraph */
bool CWaypointsNavigator::isRelativePointReachable(
    const mrpt::math::TPoint2D& wp_local_wrt_robot) const
{
  return impl_waypoint_is_reachable(wp_local_wrt_robot);
}

void CWaypointsNavigator::loadConfigFile(const mrpt::config::CConfigFileBase& c)
{
  MRPT_START

  params_waypoints_navigator.loadFromConfigFile(c, "CWaypointsNavigator");
  CAbstractNavigator::loadConfigFile(c);

  MRPT_END
}

void CWaypointsNavigator::saveConfigFile(mrpt::config::CConfigFileBase& c) const
{
  CAbstractNavigator::saveConfigFile(c);
  params_waypoints_navigator.saveToConfigFile(c, "CWaypointsNavigator");
}

void mrpt::nav::CWaypointsNavigator::TWaypointsNavigatorParams::loadFromConfigFile(
    const mrpt::config::CConfigFileBase& c, const std::string& s)
{
  MRPT_LOAD_CONFIG_VAR(max_distance_to_allow_skip_waypoint, double, c, s);
  MRPT_LOAD_CONFIG_VAR(min_timesteps_confirm_skip_waypoints, int, c, s);
  MRPT_LOAD_CONFIG_VAR_DEGREES(waypoint_angle_tolerance, c, s);
  MRPT_LOAD_CONFIG_VAR(multitarget_look_ahead, int, c, s);
  MRPT_LOAD_CONFIG_VAR(minimum_target_approach_per_step, double, c, s);
}

void mrpt::nav::CWaypointsNavigator::TWaypointsNavigatorParams::saveToConfigFile(
    mrpt::config::CConfigFileBase& c, const std::string& s) const
{
  MRPT_SAVE_CONFIG_VAR_COMMENT(
      max_distance_to_allow_skip_waypoint,
      "Max distance to `foresee` waypoints [meters]. (<0: unlimited)");
  MRPT_SAVE_CONFIG_VAR_COMMENT(
      min_timesteps_confirm_skip_waypoints,
      "Min timesteps a `future` waypoint must be seen as reachable to become "
      "the active one.");
  MRPT_SAVE_CONFIG_VAR_DEGREES_COMMENT(
      "waypoint_angle_tolerance", waypoint_angle_tolerance,
      "Angular error tolerance for waypoints with an assigned heading [deg] "
      "(Default: 5 deg)");
  MRPT_SAVE_CONFIG_VAR_COMMENT(
      multitarget_look_ahead,
      ">=0 number of waypoints to forward to the underlying navigation "
      "engine, to ease obstacles avoidance when a waypoint is blocked "
      "(Default=0 : none)");
  MRPT_SAVE_CONFIG_VAR(minimum_target_approach_per_step, c, s);
}

/** \callergraph */
bool CWaypointsNavigator::checkHasReachedTarget(const double targetDist) const
{
  bool ret;
  const TWaypointStatus* wp = nullptr;
  const auto& wps = m_waypoint_nav_status;
  if (m_navigationParams->target.targetIsIntermediaryWaypoint)
  {
    ret = false;
  }
  else if (wps.timestamp_nav_started != INVALID_TIMESTAMP)
  {
    wp = (!wps.waypoints.empty() && wps.waypoint_index_current_goal >= 0 &&
          wps.waypoint_index_current_goal < (int)wps.waypoints.size())
             ? &wps.waypoints[wps.waypoint_index_current_goal]
             : nullptr;
    ret = (wp == nullptr && targetDist <= m_navigationParams->target.targetAllowedDistance) ||
          (wp->reached);
  }
  else
  {
    ret = (targetDist <= m_navigationParams->target.targetAllowedDistance);
  }
  MRPT_LOG_DEBUG_STREAM(
      "CWaypointsNavigator::checkHasReachedTarget() called "
      "with targetDist="
      << targetDist << " return=" << ret << " waypoint: "
      << (wp == nullptr ? std::string("") : wp->getAsText()) << "wps.timestamp_nav_started="
      << (wps.timestamp_nav_started == INVALID_TIMESTAMP
              ? "INVALID_TIMESTAMP"
              : mrpt::system::dateTimeLocalToString(wps.timestamp_nav_started)));
  return ret;
}
