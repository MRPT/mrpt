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

using namespace mrpt::nav;
using namespace std;

CWaypointsNavigator::CWaypointsNavigator(CRobot2NavInterface &robot_if) :
	CAbstractNavigator(robot_if),
	MAX_DISTANCE_TO_ALLOW_SKIP_WAYPOINT(-1.0),
	MIN_TIMESTEPS_CONFIRM_SKIP_WAYPOINTS(1)
{
}

CWaypointsNavigator::~CWaypointsNavigator()
{
}

void CWaypointsNavigator::navigateWaypoints( const TWaypointSequence & nav_request )
{
	MRPT_START

	mrpt::synch::CCriticalSectionLocker csl(&m_nav_waypoints_cs);

	m_waypoint_nav_status = TWaypointStatusSequence();
	m_waypoint_nav_status.timestamp_nav_started = mrpt::system::now();

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

void CWaypointsNavigator::navigationStep()
{
	MRPT_START

	using mrpt::utils::square;

	// --------------------------------------
	//     Waypoint navigation algorithm
	// --------------------------------------
	{
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

		if (wps.waypoint_index_current_goal>=0 && 
			robot_move_seg.distance( wps.waypoints[wps.waypoint_index_current_goal].target ) < wps.waypoints[wps.waypoint_index_current_goal].allowed_distance )
		{
			wps.waypoints[wps.waypoint_index_current_goal].reached = true;
			m_robot.sendWaypointReachedEvent(wps.waypoint_index_current_goal);

			// Was this the final goal??
			if ( wps.waypoint_index_current_goal < int(wps.waypoints.size()-1) ) {
				wps.waypoint_index_current_goal++;
			}
			else {
				wps.final_goal_reached = true;
			}
		}

		// 2) More advanced policy: if available, use children class methods to decide 
		//     which is the best candidate for the next waypoint, if we can skip current one:
		if (!wps.final_goal_reached && wps.waypoint_index_current_goal >= 0)
		{
			const mrpt::poses::CPose2D robot_pose(m_curPoseVel.pose);
			int most_advanced_wp = wps.waypoint_index_current_goal;
			const int most_advanced_wp_at_begin = most_advanced_wp;

			for (int idx=wps.waypoint_index_current_goal;idx<(int)wps.waypoints.size();idx++)
			{
				if (idx<0) continue;

				// Is it reachable?
				mrpt::math::TPoint2D wp_local_wrt_robot;
				robot_pose.inverseComposePoint(wps.waypoints[idx].target, wp_local_wrt_robot);

				if (MAX_DISTANCE_TO_ALLOW_SKIP_WAYPOINT>0 && wp_local_wrt_robot.norm()>MAX_DISTANCE_TO_ALLOW_SKIP_WAYPOINT)
					continue; // Skip this one, it is too far away

				const bool is_reachable = this->impl_waypoint_is_reachable(wp_local_wrt_robot);

				if (is_reachable) {
					// Robustness filter: only skip to a future waypoint if it is seen as "reachable" during 
					// a given number of timesteps:
					if (++wps.waypoints[idx].counter_seen_reachable > MIN_TIMESTEPS_CONFIRM_SKIP_WAYPOINTS) {
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
					wps.waypoints[k].reached = true;
					m_robot.sendWaypointReachedEvent(k);
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
			TWaypointStatus &wp = wps.waypoints[wps.waypoint_index_current_goal];
			const bool is_final_wp =  ( (wps.waypoint_index_current_goal+1)==int(wps.waypoints.size()) );
			
			m_robot.sendNewWaypointTargetEvent(wps.waypoint_index_current_goal);

			CAbstractNavigator::TNavigationParams nav_cmd;
			nav_cmd.target.x = wp.target.x;
			nav_cmd.target.y = wp.target.y;
			nav_cmd.target.phi = (wp.target_heading!=TWaypoint::INVALID_NUM ? wp.target_heading : .0);
			nav_cmd.targetAllowedDistance = wp.allowed_distance;
			nav_cmd.targetIsRelative = false;
			nav_cmd.targetIsIntermediaryWaypoint = !is_final_wp;

			this->navigate( &nav_cmd );
		}
	}
	}

	// Note: navigationStep() called *after* waypoints part to get end-of-navigation events *after*
	//       waypoints-related events:

	// Call base navigation step to execute one-single waypoint navigation, as usual:
	CAbstractNavigator::navigationStep();  // This internally locks "m_nav_cs"

	MRPT_END
}

void CWaypointsNavigator::loadWaypointsParamsConfigFile(const mrpt::utils::CConfigFileBase &cfg, const std::string &sectionName)
{
	MRPT_LOAD_CONFIG_VAR(MAX_DISTANCE_TO_ALLOW_SKIP_WAYPOINT,  double,   cfg, sectionName);
	MRPT_LOAD_CONFIG_VAR(MIN_TIMESTEPS_CONFIRM_SKIP_WAYPOINTS, int, cfg, sectionName);
}

void CWaypointsNavigator::onStartNewNavigation()
{

}

bool CWaypointsNavigator::isRelativePointReachable(const mrpt::math::TPoint2D &wp_local_wrt_robot) const
{
	return impl_waypoint_is_reachable(wp_local_wrt_robot);
}
