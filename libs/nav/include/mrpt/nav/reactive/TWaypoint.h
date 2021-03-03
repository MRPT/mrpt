/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/img/TColor.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/opengl/opengl_frwds.h>
#include <mrpt/system/datetime.h>

#include <any>
#include <optional>
#include <string>
#include <vector>

namespace mrpt::nav
{
/** A single waypoint within TWaypointSequence. \ingroup nav_reactive */
struct TWaypoint
{
	/** Ctor with default values */
	TWaypoint() = default;

	TWaypoint(
		double target_x, double target_y, double allowed_distance,
		bool allow_skip = true,
		std::optional<double> target_heading_ = std::nullopt,
		double speed_ratio_ = 1.0);

	/** [Must be set by the user] Coordinates of desired target location
	 * (world/global coordinates).
	 * \sa target_heading */
	mrpt::math::TPoint2D target{INVALID_NUM, INVALID_NUM};

	/** [Default=any heading] Optionally, set to the desired orientation
	 * [radians]
	 * of the robot at this waypoint. Some navigator implementations may ignore
	 * this preferred heading anyway, read the docs of each implementation to
	 * find it out. */
	std::optional<double> target_heading;

	/** (Default="map") Frame ID in which target is given. Optional, use only
	 * for submapping applications. */
	std::string target_frame_id = "map";

	/** [Must be set by the user] How close should the robot get to this
	 * waypoint for it to be considered reached. */
	double allowed_distance{INVALID_NUM};

	/** (Default=1.0) Desired robot speed at the target, as a ratio of the full
	 * robot speed. That is: speed_ratio=1 means that the user wants the robot
	 * to navigate to the target and smoothly continue to the next one when
	 * reached. speed_ratio=0 on the other hand means that the robot should
	 * approach this waypoint slowing down and end up totally stopped.
	 */
	double speed_ratio = 1.0;

	/** [Default=true] Whether it is allowed to the navigator to proceed to a
	 * more advanced waypoint
	 * in the sequence if it determines that it is easier to skip this one
	 * (e.g. it seems blocked by dynamic obstacles).
	 * This value is ignored for the last waypoint in a sequence, since it is
	 * always considered to be the
	 * ultimate goal and hence not subject to be skipped.
	 */
	bool allow_skip = true;

	/** Check whether all the minimum mandatory fields have been filled by the
	 * user. */
	bool isValid() const;

	/** get in human-readable format */
	std::string getAsText() const;

	/** Any user-stored custom data */
	std::any user_data;

	/** The default value of fields (used to detect non-set values) */
	static constexpr int INVALID_NUM{-100000};
};

/** used in getAsOpenglVisualization() */
struct TWaypointsRenderingParams
{
	TWaypointsRenderingParams();

	double outter_radius{.3}, inner_radius{.2};
	double outter_radius_non_skippable{.3}, inner_radius_non_skippable{.0};
	double outter_radius_reached{.2}, inner_radius_reached{.1};
	double heading_arrow_len{1.0};
	mrpt::img::TColor color_regular, color_current_goal, color_reached;
	bool show_labels{true};
};

/** The struct for requesting navigation requests for a sequence of waypoints.
 * Used in CWaypointsNavigator::navigateWaypoints().
 * Users can directly fill in the list of waypoints manipulating the public
 * field `waypoints`.
 *  \ingroup nav_reactive */
struct TWaypointSequence
{
	std::vector<TWaypoint> waypoints;

	void clear() { waypoints.clear(); }
	/** Ctor with default values */
	TWaypointSequence();
	/** Gets navigation params as a human-readable format */
	std::string getAsText() const;
	/** Renders the sequence of waypoints (previous contents of `obj` are
	 * cleared) */
	void getAsOpenglVisualization(
		mrpt::opengl::CSetOfObjects& obj,
		const mrpt::nav::TWaypointsRenderingParams& params =
			mrpt::nav::TWaypointsRenderingParams()) const;
	/** Saves waypoints to a config file section */
	void save(mrpt::config::CConfigFileBase& c, const std::string& s) const;
	/** Loads waypoints to a config file section */
	void load(const mrpt::config::CConfigFileBase& c, const std::string& s);
};

/** A waypoint with an execution status. \ingroup nav_reactive */
struct TWaypointStatus : public TWaypoint
{
	TWaypointStatus() = default;

	/** Whether this waypoint has been reached already (to within the allowed
	 * distance as per user specifications) or skipped. */
	bool reached{false};

	/** If `reached==true` this boolean tells whether the waypoint was
	 * physically reached (false) or marked as reached because it was skipped
	 * (true). */
	bool skipped{false};

	/** Timestamp of when this waypoint was reached. (Default=INVALID_TIMESTAMP
	 * means not reached so far) */
	mrpt::system::TTimeStamp timestamp_reach = INVALID_TIMESTAMP;

	/** (Initialized to 0 automatically) How many times this waypoint has been
	 * seen as "reachable" before it being the current active waypoint. */
	int counter_seen_reachable{0};

	/** Any user-stored custom status data */
	std::any user_status_data;

	/** Only copies the base class TWaypoint data fields */
	TWaypointStatus& operator=(const TWaypoint& wp);

	/** Gets navigation params as a human-readable format */
	std::string getAsText() const;
};

/** The struct for querying the status of waypoints navigation. Used in
 * CWaypointsNavigator::getWaypointNavStatus().
 *  \ingroup nav_reactive */
struct TWaypointStatusSequence
{
	TWaypointStatusSequence() = default;

	/** Waypoints parameters and status (reached, skipped, etc.) */
	std::vector<TWaypointStatus> waypoints;

	/** Timestamp of user navigation command. */
	mrpt::system::TTimeStamp timestamp_nav_started = INVALID_TIMESTAMP;

	/** Whether the final waypoint has been reached successfuly. */
	bool final_goal_reached{false};

	/** Index in `waypoints` of the waypoint the navigator is currently trying
	 * to reach.
	 * This will point to the last waypoint after navigation ends successfully.
	 * Its value is `-1` if navigation has not started yet */
	int waypoint_index_current_goal{-1};

	/** Robot pose at last time step (has INVALID_NUM fields upon
	 * initialization) */
	mrpt::math::TPose2D last_robot_pose{
		TWaypoint::INVALID_NUM, TWaypoint::INVALID_NUM, TWaypoint::INVALID_NUM};

	/** Ctor with default values */
	/** Gets navigation params as a human-readable format */
	std::string getAsText() const;

	/** Renders the sequence of waypoints (previous contents of `obj` are
	 * cleared) */
	void getAsOpenglVisualization(
		mrpt::opengl::CSetOfObjects& obj,
		const mrpt::nav::TWaypointsRenderingParams& params =
			mrpt::nav::TWaypointsRenderingParams()) const;
};
}  // namespace mrpt::nav
