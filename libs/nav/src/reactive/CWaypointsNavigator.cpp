/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "nav-precomp.h" // Precomp header

#include <mrpt/nav/reactive/CWaypointsNavigator.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/math/wrap2pi.h>

using namespace mrpt::nav;
using namespace std;

std::string CWaypointsNavigator::TNavigationParamsWaypoints::getAsText() const
{
	std::string s = TNavigationParams::getAsText();
	if (!multiple_targets.empty())
	{
		s += "multiple_targets:\n";
		int i = 0;
		for (const auto &e : multiple_targets) {
			s += mrpt::format("target[%i]:\n",i++);
			s += e.getAsText();
		}
	}
	return s;
}

bool CWaypointsNavigator::TNavigationParamsWaypoints::isEqual(const CAbstractNavigator::TNavigationParamsBase& rhs) const
{
	auto o = dynamic_cast<const CWaypointsNavigator::TNavigationParamsWaypoints*>(&rhs);
	return o!=nullptr &&
		CAbstractNavigator::TNavigationParams::isEqual(rhs) &&
		multiple_targets == o->multiple_targets;
}

CWaypointsNavigator::CWaypointsNavigator(CRobot2NavInterface &robot_if) :
	CAbstractNavigator(robot_if),
	m_was_aligning(false),
	m_is_aligning(false),
	m_last_alignment_cmd(mrpt::system::now())
{
}

CWaypointsNavigator::~CWaypointsNavigator()
{
}

void CWaypointsNavigator::navigateWaypoints( const TWaypointSequence & nav_request )
{
	MRPT_START

	mrpt::synch::CCriticalSectionLocker csl(&m_nav_waypoints_cs);

	m_was_aligning = false;
	m_waypoint_nav_status = TWaypointStatusSequence();
	m_waypoint_nav_status.timestamp_nav_started = mrpt::system::now();
	m_pending_events.clear();

	const size_t N = nav_request.waypoints.size();
	ASSERTMSG_(N>0,"List of waypoints is empty!");

	m_waypoint_nav_status.waypoints.resize(N);
	// Copy waypoints fields data, leave status fields to defaults:
	for (size_t i=0;i<N;i++)
	{
		ASSERT_( nav_request.waypoints[i].isValid() );
		m_waypoint_nav_status.waypoints[i] = nav_request.waypoints[i];
	}

	m_waypoint_nav_status.waypoint_index_current_goal = -1;  // Not started yet.

	// The main loop navigationStep() will iterate over waypoints and send them to navigate()
	MRPT_END
}

void CWaypointsNavigator::getWaypointNavStatus(TWaypointStatusSequence & out_nav_status) const
{
	// No need to lock mutex...
	out_nav_status = m_waypoint_nav_status;
}

void CWaypointsNavigator::cancel()
{
	{
		mrpt::synch::CCriticalSectionLocker csl(&m_nav_waypoints_cs);
		m_waypoint_nav_status = TWaypointStatusSequence();
	}
	CAbstractNavigator::cancel();
}

void CWaypointsNavigator::waypoints_navigationStep()
{
	MRPT_START

	using mrpt::math::square;

	// --------------------------------------
	//     Waypoint navigation algorithm
	// --------------------------------------
	m_is_aligning = false;  // the robot is aligning into a waypoint with a desired heading

	{
	mrpt::utils::CTimeLoggerEntry tle(m_timlog_delays,"CWaypointsNavigator::navigationStep()");
	mrpt::synch::CCriticalSectionLocker csl(&m_nav_waypoints_cs);

	TWaypointStatusSequence &wps = m_waypoint_nav_status; // shortcut to save typing

	if (wps.waypoints.empty() || wps.final_goal_reached)
	{
		// No nav request is pending or it was canceled
	}
	else
	{
		// 0) Get current robot pose:
		CAbstractNavigator::updateCurrentPoseAndSpeeds();

		// 1) default policy: go thru WPs one by one
		const int prev_wp_index = wps.waypoint_index_current_goal;

		mrpt::math::TSegment2D robot_move_seg;
		robot_move_seg.point1.x = m_curPoseVel.pose.x;
		robot_move_seg.point1.y = m_curPoseVel.pose.y;
		if (wps.last_robot_pose.x==TWaypoint::INVALID_NUM)
		{
			robot_move_seg.point2 = robot_move_seg.point1;
		}
		else
		{
			robot_move_seg.point2.x = wps.last_robot_pose.x;
			robot_move_seg.point2.y = wps.last_robot_pose.y;
		}
		wps.last_robot_pose = m_curPoseVel.pose; // save for next iters

		if (wps.waypoint_index_current_goal >= 0)
		{
			auto &wp = wps.waypoints[wps.waypoint_index_current_goal];
			const double dist2target = robot_move_seg.distance(wp.target);
			if (dist2target < wp.allowed_distance || m_was_aligning /* we were already aligning at a WP */)
			{
				bool consider_wp_reached = false;
				if (wp.target_heading == TWaypoint::INVALID_NUM)
				{
					consider_wp_reached = true;
				}
				else
				{
					// Handle pure-rotation robot interface to honor target_heading
					const double ang_err = mrpt::math::angDistance(m_curPoseVel.pose.phi, wp.target_heading);
					const double tim_since_last_align = mrpt::system::timeDifference(m_last_alignment_cmd, mrpt::system::now());
					const double ALIGN_WAIT_TIME = 1.5; // seconds
					if (std::abs(ang_err) <= params_waypoints_navigator.waypoint_angle_tolerance && 
						tim_since_last_align>ALIGN_WAIT_TIME // give some time for the alignment (if supported in this robot) to finish
						)
					{
						consider_wp_reached = true;
					}
					else
					{
						m_is_aligning = true;

						if (!m_was_aligning || tim_since_last_align> ALIGN_WAIT_TIME)
						{
							// we must align:
							// Send vel_cmd to the robot:
							mrpt::kinematics::CVehicleVelCmdPtr align_cmd = m_robot.getAlignCmd(ang_err);

							MRPT_LOG_INFO_FMT(
								"[CWaypointsNavigator::navigationStep] Trying to align to heading: %.02f deg. "
								"Relative heading: %.02f deg. "
								"With motion cmd: %s",
								mrpt::utils::RAD2DEG(wp.target_heading),
								mrpt::utils::RAD2DEG(ang_err),
								align_cmd ? align_cmd->asString().c_str() : "nullptr (operation not supported by this robot)");

							if (align_cmd)
							{
								m_last_alignment_cmd = mrpt::system::now();
								this->changeSpeeds(*align_cmd);
							}
							else
							{
								this->stop(false /*not emergency*/); // In any case, do a "stop"
								consider_wp_reached = true; // this robot does not support "in place" alignment
							}
						}
						else
						{
							MRPT_LOG_THROTTLE_INFO_FMT(0.5, "[CWaypointsNavigator::navigationStep] Waiting for the robot to get aligned: current_heading=%.02f deg target_heading=%.02f deg", mrpt::utils::RAD2DEG(m_curPoseVel.pose.phi), mrpt::utils::RAD2DEG(wp.target_heading) );
						}
					}
				}

				if (consider_wp_reached)
				{
					MRPT_LOG_DEBUG_STREAM("[CWaypointsNavigator::navigationStep] Waypoint " <<
						(wps.waypoint_index_current_goal + 1) << "/" << wps.waypoints.size() << " reached."
						" segment-to-target dist: " << dist2target << ", allowed_dist: " << wp.allowed_distance
					);

					wp.reached = true;
					wp.skipped = false;
					wp.timestamp_reach = mrpt::system::now();
					{
						TPendingEvent ev;
						ev.event_wp_reached = true;
						ev.event_wp_reached_index = wps.waypoint_index_current_goal;
						ev.event_wp_reached_reached = true /* reason: really reached*/;
						m_pending_events.push_back(ev);
					}

					// Was this the final goal??
					if (wps.waypoint_index_current_goal < int(wps.waypoints.size() - 1)) {
						wps.waypoint_index_current_goal++;
					}
					else {
						wps.final_goal_reached = true;

						// Make sure the end-navigation event is issued,
						// navigation state switches to IDLE, etc:
						this->performNavigationStepNavigating(false);
					}
				}
			}
		}

		// 2) More advanced policy: if available, use children class methods to decide
		//     which is the best candidate for the next waypoint, if we can skip current one:
		if (!wps.final_goal_reached &&
			wps.waypoint_index_current_goal >= 0 &&
			wps.waypoints[wps.waypoint_index_current_goal].allow_skip
			)
		{
			const mrpt::poses::CPose2D robot_pose(m_curPoseVel.pose);
			int most_advanced_wp = wps.waypoint_index_current_goal;
			const int most_advanced_wp_at_begin = most_advanced_wp;

			for (int idx=wps.waypoint_index_current_goal;idx<(int)wps.waypoints.size();idx++)
			{
				if (idx<0) continue;
				if (wps.waypoints[idx].reached) continue;

				// Is it reachable?
				mrpt::math::TPoint2D wp_local_wrt_robot;
				robot_pose.inverseComposePoint(wps.waypoints[idx].target, wp_local_wrt_robot);

				if (params_waypoints_navigator.max_distance_to_allow_skip_waypoint>0 && wp_local_wrt_robot.norm()>params_waypoints_navigator.max_distance_to_allow_skip_waypoint)
					continue; // Skip this one, it is too far away

				const bool is_reachable = this->impl_waypoint_is_reachable(wp_local_wrt_robot);

				if (is_reachable) {
					// Robustness filter: only skip to a future waypoint if it is seen as "reachable" during
					// a given number of timesteps:
					if (++wps.waypoints[idx].counter_seen_reachable > params_waypoints_navigator.min_timesteps_confirm_skip_waypoints) {
						most_advanced_wp = idx;
					}
				}

				// Is allowed to skip it?
				if (!wps.waypoints[idx].allow_skip)
					break; // Do not keep trying, since we are now allowed to skip this one.
			}

			if (most_advanced_wp>=0) {
				wps.waypoint_index_current_goal = most_advanced_wp;
				for (int k=most_advanced_wp_at_begin;k<most_advanced_wp;k++) {
					auto &wp = wps.waypoints[k];
					wp.reached = true;
					wp.skipped = true;
					wp.timestamp_reach = mrpt::system::now();

					{
						TPendingEvent ev;
						ev.event_wp_reached = true;
						ev.event_wp_reached_index = k;
						ev.event_wp_reached_reached = false /* reason: skipped */;
						m_pending_events.push_back(ev);
					}
				}
			}
		}

		// Still not started and no better guess? Start with the first waypoint:
		if (wps.waypoint_index_current_goal<0)
			wps.waypoint_index_current_goal = 0;

		// 3) Should I request a new (single target) navigation command?
		//    Only if the temporary goal changed:
		if (wps.waypoint_index_current_goal>=0 && prev_wp_index!=wps.waypoint_index_current_goal)
		{
			ASSERT_( wps.waypoint_index_current_goal < int(wps.waypoints.size()) );
			ASSERT_( params_waypoints_navigator.multitarget_look_ahead>=0);

			// Notify we have a new "current waypoint"
			{
				TPendingEvent ev;
				ev.event_new_wp = true;
				ev.event_new_wp_index = wps.waypoint_index_current_goal;
				m_pending_events.push_back(ev);
			}

			// Send the current targets + "multitarget_look_ahead" additional ones to help the local planner.
			CWaypointsNavigator::TNavigationParamsWaypoints nav_cmd;

			// Check skippable flag while traversing from current wp forward "multitarget_look_ahead" steps:
			int wp_last_idx = wps.waypoint_index_current_goal;
			for (int nstep = 0; wp_last_idx<int(wps.waypoints.size()) - 1 && nstep < params_waypoints_navigator.multitarget_look_ahead; ++nstep)
			{
				if (!m_waypoint_nav_status.waypoints[wp_last_idx].allow_skip)
					break;
				wp_last_idx++;
			}

			for (int wp_idx = wps.waypoint_index_current_goal; wp_idx <= wp_last_idx; wp_idx++)
			{
				TWaypointStatus &wp = wps.waypoints[wp_idx];
				const bool is_final_wp = ((wp_idx + 1) == int(wps.waypoints.size()));

				CAbstractNavigator::TargetInfo ti;

				ti.target_coords.x = wp.target.x;
				ti.target_coords.y = wp.target.y;
				ti.target_coords.phi = (wp.target_heading != TWaypoint::INVALID_NUM ? wp.target_heading : .0);
				ti.target_frame_id = wp.target_frame_id;
				ti.targetAllowedDistance = wp.allowed_distance;
				ti.targetIsRelative = false;
				ti.targetIsIntermediaryWaypoint = !is_final_wp;
				ti.targetDesiredRelSpeed = (is_final_wp || (wp.target_heading != TWaypoint::INVALID_NUM)) ? params_waypoints_navigator.rel_speed_for_stop_waypoints : 1.;

				// For backwards compat. with single-target code, write single target info too for the first, next, waypoint:
				if (wp_idx == wps.waypoint_index_current_goal) {
					nav_cmd.target = ti;
				}
				// Append to list of targets:
				nav_cmd.multiple_targets.emplace_back(ti);
			}
			this->navigate( &nav_cmd );

			MRPT_LOG_DEBUG_STREAM( "[CWaypointsNavigator::navigationStep] Active waypoint changed. Current status:\n" << this->getWaypointNavStatus().getAsText());
		}
	}
	}

	// Note: navigationStep() called *after* waypoints part to get end-of-navigation events *after*
	//       waypoints-related events:

	m_was_aligning = m_is_aligning; // Let the next timestep know about this

	MRPT_END
}


void CWaypointsNavigator::navigationStep()
{
	MRPT_START
	m_is_aligning = false;  // the robot is aligning into a waypoint with a desired heading

	// State can be NAVIGATING if we are already heading to a waypoint, 
	// or IDLE if navigateWaypoints() was called and this is the first
	// navStep() afterwards:
	if (m_navigationState == NAVIGATING || m_navigationState == IDLE)
	{
		waypoints_navigationStep();
	}

	// Call base navigation step to execute one-single waypoint navigation, as usual:
	if (!m_is_aligning)
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

void CWaypointsNavigator::onStartNewNavigation()
{
}

bool CWaypointsNavigator::isRelativePointReachable(const mrpt::math::TPoint2D &wp_local_wrt_robot) const
{
	return impl_waypoint_is_reachable(wp_local_wrt_robot);
}

void CWaypointsNavigator::loadConfigFile(const mrpt::utils::CConfigFileBase &c)
{
	MRPT_START

	params_waypoints_navigator.loadFromConfigFile(c, "CWaypointsNavigator");
	CAbstractNavigator::loadConfigFile(c);

	MRPT_END
}

void CWaypointsNavigator::saveConfigFile(mrpt::utils::CConfigFileBase &c) const
{
	CAbstractNavigator::saveConfigFile(c);
	params_waypoints_navigator.saveToConfigFile(c, "CWaypointsNavigator");
}

void mrpt::nav::CWaypointsNavigator::TWaypointsNavigatorParams::loadFromConfigFile(const mrpt::utils::CConfigFileBase & c, const std::string & s)
{
	MRPT_LOAD_CONFIG_VAR(max_distance_to_allow_skip_waypoint, double, c, s);
	MRPT_LOAD_CONFIG_VAR(min_timesteps_confirm_skip_waypoints, int, c, s);
	MRPT_LOAD_CONFIG_VAR_DEGREES(waypoint_angle_tolerance, c, s);
	MRPT_LOAD_CONFIG_VAR(rel_speed_for_stop_waypoints, double, c, s);
	MRPT_LOAD_CONFIG_VAR(multitarget_look_ahead, int, c, s);
}

void mrpt::nav::CWaypointsNavigator::TWaypointsNavigatorParams::saveToConfigFile(mrpt::utils::CConfigFileBase & c, const std::string & s) const
{
	MRPT_SAVE_CONFIG_VAR_COMMENT(max_distance_to_allow_skip_waypoint, "Max distance to `foresee` waypoints [meters]. (<0: unlimited)");
	MRPT_SAVE_CONFIG_VAR_COMMENT(min_timesteps_confirm_skip_waypoints, "Min timesteps a `future` waypoint must be seen as reachable to become the active one.");
	MRPT_SAVE_CONFIG_VAR_DEGREES_COMMENT("waypoint_angle_tolerance", waypoint_angle_tolerance, "Angular error tolerance for waypoints with an assigned heading [deg] (Default: 5 deg)");
	MRPT_SAVE_CONFIG_VAR_COMMENT(rel_speed_for_stop_waypoints, "[0,1] Relative speed when aiming at a stop-point waypoint (Default=0.10)");
	MRPT_SAVE_CONFIG_VAR_COMMENT(multitarget_look_ahead, ">=0 number of waypoints to forward to the underlying navigation engine, to ease obstacles avoidance when a waypoint is blocked (Default=0 : none)");
}

CWaypointsNavigator::TWaypointsNavigatorParams::TWaypointsNavigatorParams() :
	max_distance_to_allow_skip_waypoint(-1.0),
	min_timesteps_confirm_skip_waypoints(1),
	waypoint_angle_tolerance( mrpt::utils::DEG2RAD(5.0) ),
	rel_speed_for_stop_waypoints(0.10),
	multitarget_look_ahead(0)
{
}

bool CWaypointsNavigator::checkHasReachedTarget(const double targetDist) const
{
	if (targetDist > m_navigationParams->target.targetAllowedDistance)
		return false;
	if (m_navigationParams->target.targetIsIntermediaryWaypoint)
		return false;

	const auto &wps = m_waypoint_nav_status;
	const TWaypointStatus *wp = (!wps.waypoints.empty() &&
		wps.waypoint_index_current_goal >= 0 && 
		wps.waypoint_index_current_goal < (int)wps.waypoints.size()
		)
		?
		&wps.waypoints[wps.waypoint_index_current_goal]
		:
		nullptr;

	return ( wp==nullptr || wp->reached);
}
