/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
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

		bool isValid() const; //!< Check whether all the minimum mandatory fields have been filled by the user.
		TWaypoint(); //!< Ctor with default values
		std::string getAsText() const; //!< get in human-readable format
	};

	/** The struct for requesting navigation requests for a sequence of waypoints.
	 * Used in CAbstractWaypointsNavigator::navigateWaypoints(). 
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

		/** Whether this waypoint has been definitively skipped, that is, whether a more 
		  * advanced waypoint has been reached and no further attempt will be done to reach this one. */
		bool skipped;

		TWaypointStatus();
		TWaypointStatus & operator =(const TWaypoint &wp);
		std::string getAsText() const; //!< Gets navigation params as a human-readable format
	};

	/** The struct for querying the status of waypoints navigation. Used in CAbstractWaypointsNavigator::getWaypointNavStatus().
	 *  \ingroup nav_reactive */
	struct NAV_IMPEXP TWaypointStatusSequence
	{
		std::vector<TWaypointStatus> waypoints; //!< Waypoints parameters and status (reached, skipped, etc.)
		mrpt::system::TTimeStamp     timestamp_nav_started; //!< Timestamp of user navigation command.
		bool final_goal_reached; //!< Whether the final waypoint has been reached successfuly.
		int  waypoint_index_current_goal;  //!< Index in `waypoints` of the waypoint the navigator is currently trying to reach, or `-1` if navigation already ended for any reason.

		TWaypointStatusSequence(); //!< Ctor with default values
		std::string getAsText() const; //!< Gets navigation params as a human-readable format
	};

  }
}

