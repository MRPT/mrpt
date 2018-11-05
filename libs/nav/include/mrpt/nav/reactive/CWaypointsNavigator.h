/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/nav/reactive/CAbstractNavigator.h>
#include <mrpt/nav/reactive/TWaypoint.h>

namespace mrpt::nav
{
/** This class extends `CAbstractNavigator` with the capability of following a
 * list of waypoints. By default, waypoints are followed one by one,
 *  but, if they are tagged with `allow_skip=true` **and** the derived navigator
 * class supports it, the navigator may choose to skip some to
 *  make a smoother, safer and shorter navigation.
 *
 * Waypoints have an optional `target_heading` field, which will be honored only
 * for waypoints that are skipped, and if the underlying robot
 * interface supports the pure-rotation methods.
 *
 * Notes on navigation status and event dispatchment:
 *  - Navigation state may briefly pass by the `IDLE` status between a waypoint
 * is reached and a new navigation command is issued towards the next waypoint.
 *  - `sendNavigationEndEvent()` will be called only when the last waypoint is
 * reached.
 *  - Reaching an intermediary waypoint (or skipping it if considered so by the
 * navigator) generates a call to `sendWaypointReachedEvent()` instead.
 *
 * \sa Base class CAbstractNavigator, CWaypointsNavigator::navigateWaypoints(),
 * and derived classes.
 *  \ingroup nav_reactive
 */
class CWaypointsNavigator : public mrpt::nav::CAbstractNavigator
{
   public:
	/** The struct for configuring navigation requests to CWaypointsNavigator
	 * and derived classes. */
	struct TNavigationParamsWaypoints
		: public CAbstractNavigator::TNavigationParams
	{
		/** If not empty, this will prevail over the base class single goal
		 * target.
		 * Semantic is: any of these targets will be good for heading the robot
		 * towards them,
		 * but the priority is for the latest ones in the sequence. */
		std::vector<mrpt::nav::CAbstractNavigator::TargetInfo> multiple_targets;

		std::string getAsText() const override;
		std::unique_ptr<TNavigationParams> clone() const override
		{
			return std::unique_ptr<TNavigationParams>(
				new TNavigationParamsWaypoints(*this));
		}

	   protected:
		bool isEqual(
			const CAbstractNavigator::TNavigationParamsBase& o) const override;
	};

	/** ctor */
	CWaypointsNavigator(CRobot2NavInterface& robot_interface_impl);
	/** dtor */
	~CWaypointsNavigator() override;

	// Overriden to call the general navigationStep(), plus waypoint selection
	// logic.
	void navigationStep() override;
	/** Cancel current navegation. */
	void cancel() override;

	/** \name Waypoint navigation control API
	 * @{ */

	/** Waypoint navigation request. This immediately cancels any other previous
	 * on-going navigation.
	 * \sa CAbstractNavigator::navigate() for single waypoint navigation
	 * requests.
	 */
	virtual void navigateWaypoints(const TWaypointSequence& nav_request);

	/** Get a copy of the control structure which describes the progress status
	 * of the waypoint navigation. */
	virtual void getWaypointNavStatus(
		TWaypointStatusSequence& out_nav_status) const;

	/** Get a copy of the control structure which describes the progress status
	 * of the waypoint navigation. */
	TWaypointStatusSequence getWaypointNavStatus() const
	{
		TWaypointStatusSequence nav_status;
		this->getWaypointNavStatus(nav_status);
		return nav_status;
	}
	/** @}*/

	/** Returns `true` if, according to the information gathered at the last
	 * navigation step,
	 * there is a free path to the given point; `false` otherwise: if way is
	 * blocked or there is missing information,
	 * the point is out of range for the existing PTGs, etc. */
	bool isRelativePointReachable(
		const mrpt::math::TPoint2D& wp_local_wrt_robot) const;

	struct TWaypointsNavigatorParams : public mrpt::config::CLoadableOptions
	{
		/** In meters. <0: unlimited */
		double max_distance_to_allow_skip_waypoint{-1.0};
		/** How many times shall a future waypoint be seen as reachable to skip
		 * to it (Default: 1) */
		int min_timesteps_confirm_skip_waypoints{1};
		/** [rad] Angular error tolerance for waypoints with an assigned heading
		 * (Default: 5 deg) */
		double waypoint_angle_tolerance;
		/** [0,1] Relative speed when aiming at a stop-point waypoint
		 * (Default=0.10) */
		double rel_speed_for_stop_waypoints{0.10};
		/** >=0 number of waypoints to forward to the underlying navigation
		 * engine, to ease obstacles avoidance when a waypoint is blocked
		 * (Default=0 : none). */
		int multitarget_look_ahead{0};

		void loadFromConfigFile(
			const mrpt::config::CConfigFileBase& c,
			const std::string& s) override;
		void saveToConfigFile(
			mrpt::config::CConfigFileBase& c,
			const std::string& s) const override;
		TWaypointsNavigatorParams();
	};

	TWaypointsNavigatorParams params_waypoints_navigator;

	void loadConfigFile(const mrpt::config::CConfigFileBase& c)
		override;  // See base class docs!
	void saveConfigFile(mrpt::config::CConfigFileBase& c)
		const override;  // See base class docs!

   protected:
	/** The latest waypoints navigation command and the up-to-date control
	 * status. */
	TWaypointStatusSequence m_waypoint_nav_status;
	std::recursive_mutex m_nav_waypoints_cs;

	/** Implements the way to waypoint is free function in children classes:
	 * `true` must be returned
	 * if, according to the information gathered at the last navigation step,
	 * there is a free path to
	 * the given point; `false` otherwise: if way is blocked or there is
	 * missing information, the point is out of range, etc. */
	virtual bool impl_waypoint_is_reachable(
		const mrpt::math::TPoint2D& wp_local_wrt_robot) const = 0;

	void onStartNewNavigation() override;

	void onNavigateCommandReceived() override;

	bool checkHasReachedTarget(const double targetDist) const override;
	/** The waypoints-specific part of navigationStep() */
	virtual void waypoints_navigationStep();

	bool waypoints_isAligning() const { return m_is_aligning; }
	/** Whether the last timestep was "is_aligning" in a waypoint with heading
	 */
	bool m_was_aligning;
	bool m_is_aligning;
	mrpt::system::TTimeStamp m_last_alignment_cmd;
};
}  // namespace mrpt::nav
