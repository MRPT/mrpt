/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "nav-precomp.h" // Precomp header

#include <mrpt/nav/reactive/CAbstractWaypointsNavigator.h>

using namespace mrpt::nav;
using namespace std;

CAbstractWaypointsNavigator::CAbstractWaypointsNavigator(CRobot2NavInterface &robot_if) :
	CAbstractNavigator(robot_if)
{
}

CAbstractWaypointsNavigator::~CAbstractWaypointsNavigator()
{
}

void CAbstractWaypointsNavigator::navigateWaypoints( const TWaypointSequence & nav_request )
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

void CAbstractWaypointsNavigator::getWaypointNavStatus(TWaypointStatusSequence & out_nav_status) const
{
	// No need to lock mutex...
	out_nav_status = m_waypoint_nav_status;
}

void CAbstractWaypointsNavigator::cancel()
{
	{
		mrpt::synch::CCriticalSectionLocker csl(&m_nav_waypoints_cs);
		m_waypoint_nav_status = TWaypointStatusSequence();
	}
	CAbstractNavigator::cancel();
}

void CAbstractWaypointsNavigator::navigationStep()
{
	MRPT_START

	using mrpt::utils::square;

	const TState prevState = m_navigationState;

	// Call base navigation step to execute one-single waypoint navigation, as usual:
	CAbstractNavigator::navigationStep();  // This internally locks "m_nav_cs"

	// --------------------------------------
	//     Waypoint navigation algorithm
	// --------------------------------------
	{
		mrpt::synch::CCriticalSectionLocker csl(&m_nav_waypoints_cs);

		TWaypointStatusSequence &wps = m_waypoint_nav_status; // shortcut to save typing

		if ( wps.waypoints.empty() || wps.final_goal_reached )
			return; // No nav request is pending or it was canceled

		// 0) Get current robot pose:
		mrpt::math::TPose2D  currentPose;
		mrpt::math::TTwist2D cur_vel;
		if ( !m_robot.getCurrentPoseAndSpeeds(currentPose, cur_vel) ) {
			doEmergencyStop("\n[CAbstractWaypointsNavigator] Error querying current robot pose.\n");
			return;
		}

		// 1) default policy: go thru WPs one by one
		const int prev_wp_index = wps.waypoint_index_current_goal;

		if (wps.waypoint_index_current_goal>=0 && 
			std::sqrt( 
			square(wps.waypoints[wps.waypoint_index_current_goal].target.x - currentPose.x)+ 
			square(wps.waypoints[wps.waypoint_index_current_goal].target.y - currentPose.y)
			) < wps.waypoints[wps.waypoint_index_current_goal].allowed_distance )
		{
			wps.waypoints[wps.waypoint_index_current_goal].reached = true;
			MRPT_TODO("Callback for waypoint reached.")

			// Was this the final goal??
			if ( wps.waypoint_index_current_goal < (wps.waypoints.size()-1) ) {
				wps.waypoint_index_current_goal++;
			}
			else {
				wps.final_goal_reached = true;
			}
		}

		// 2) More advance policy: if available, use children class methods to decide 
		//     which is the best candidate for the next waypoint, if we can skip current one:
		if (!wps.final_goal_reached)
		{
			MRPT_TODO("to do")
		}

		// Still not started and no better guess? Start with the first waypoint:
		if (wps.waypoint_index_current_goal<0)
			wps.waypoint_index_current_goal = 0;

		// 3) Should I request a new (single target) navigation command? 
		//    Only if the temporary goal changed:
		if (prev_wp_index!=wps.waypoint_index_current_goal)
		{
			ASSERT_( wps.waypoint_index_current_goal < wps.waypoints.size() );
			TWaypointStatus &wp = wps.waypoints[wps.waypoint_index_current_goal];
			MRPT_TODO("Callback for heading new waypoint.")

			CAbstractNavigator::TNavigationParams nav_cmd;
			nav_cmd.target.x = wp.target.x;
			nav_cmd.target.y = wp.target.y;
			nav_cmd.target.phi = (wp.target_heading!=TWaypoint::INVALID_NUM ? wp.target_heading : .0);
			nav_cmd.targetAllowedDistance = wp.allowed_distance;
			nav_cmd.targetIsRelative = false;
			this->navigate( &nav_cmd );
		}
	}

	MRPT_END
}



