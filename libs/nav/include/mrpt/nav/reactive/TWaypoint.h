/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/math/lightweight_geom_data.h>
#include <mrpt/system/datetime.h>
#include <vector>
#include <string>

#include <mrpt/nav/link_pragmas.h>

namespace mrpt
{
  namespace nav
  {
	/** A single waypoint within TWaypointSequence. \ingroup nav_reactive */
	struct NAV_IMPEXP TWaypoint
	{
		/** [Must be set by the user] Coordinates of desired target location (world/global coordinates). 
		  * \sa target_heading */
		mrpt::math::TPoint2D  target;
		/** [Default=any heading] Optionally, set to the desired orientation [radians] 
		  * of the robot at this waypoint. Some navigator implementations may ignore 
		  * this preferred heading anyway, read the docs of each implementation to find it out. */
		double  target_heading;
		double  allowed_distance; //!< [Must be set by the user] How close should the robot get to this waypoint for it to be considered reached.

		/** [Default=true] Whether it is allowed to the navigator to proceed to a more advanced waypoint 
		  * in the sequence if it determines that it is easier to skip this one (e.g. it seems blocked by dynamic obstacles).
		  * This value is ignored for the last waypoint in a sequence, since it is always considered to be the 
		  * ultimate goal and hence not subject to be skipped.
		  */
		bool    allow_skip;

		int counter_seen_reachable; //!< (Initialized to 0 automatically) How many times this waypoint has been seen as "reachable" before it being the current active waypoint.

		bool isValid() const; //!< Check whether all the minimum mandatory fields have been filled by the user.
		TWaypoint(); //!< Ctor with default values
		TWaypoint(double target_x, double target_y, double allowed_distance, bool allow_skip = true);
		std::string getAsText() const; //!< get in human-readable format

		static const double INVALID_NUM; //!< The default value of fields (used to detect non-set values)
	};

	/** The struct for requesting navigation requests for a sequence of waypoints.
	 * Used in CWaypointsNavigator::navigateWaypoints(). 
	 * Users can directly fill in the list of waypoints manipulating the public field `waypoints`.
	 *  \ingroup nav_reactive */
	struct NAV_IMPEXP TWaypointSequence
	{
		std::vector<TWaypoint> waypoints;

		void clear() { waypoints.clear(); }

		TWaypointSequence(); //!< Ctor with default values
		std::string getAsText() const; //!< Gets navigation params as a human-readable format
	};

	/** A waypoint with an execution status. \ingroup nav_reactive */
	struct NAV_IMPEXP TWaypointStatus : public TWaypoint
	{
		bool reached; //!< Whether this waypoint has been reached already (to within the allowed distance as per user specifications).
		mrpt::system::TTimeStamp  timestamp_reach; //!< Timestamp of when this waypoint was reached. (Default=INVALID_TIMESTAMP means not reached so far)

		TWaypointStatus();
		TWaypointStatus & operator =(const TWaypoint &wp);
		std::string getAsText() const; //!< Gets navigation params as a human-readable format
	};

	/** The struct for querying the status of waypoints navigation. Used in CWaypointsNavigator::getWaypointNavStatus().
	 *  \ingroup nav_reactive */
	struct NAV_IMPEXP TWaypointStatusSequence
	{
		std::vector<TWaypointStatus> waypoints; //!< Waypoints parameters and status (reached, skipped, etc.)
		mrpt::system::TTimeStamp     timestamp_nav_started; //!< Timestamp of user navigation command.
		bool final_goal_reached; //!< Whether the final waypoint has been reached successfuly.
		/** Index in `waypoints` of the waypoint the navigator is currently trying to reach.
		  * This will point to the last waypoint after navigation ends successfully. 
		  * Its value is `-1` if navigation has not started yet */
		int  waypoint_index_current_goal;

		mrpt::math::TPose2D  last_robot_pose; //!< Robot pose at last time step (has INVALID_NUM fields upon initialization)

		TWaypointStatusSequence(); //!< Ctor with default values
		std::string getAsText() const; //!< Gets navigation params as a human-readable format
	};

  }
}

