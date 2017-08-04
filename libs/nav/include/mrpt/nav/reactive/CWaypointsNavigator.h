/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/nav/reactive/CAbstractNavigator.h>
#include <mrpt/nav/reactive/TWaypoint.h>

namespace mrpt
{
  namespace nav
  {
	/** This class extends `CAbstractNavigator` with the capability of following a list of waypoints. By default, waypoints are followed one by one, 
	 *  but, if they are tagged with `allow_skip=true` **and** the derived navigator class supports it, the navigator may choose to skip some to 
	 *  make a smoother, safer and shorter navigation.
	 *
	 * Waypoints have an optional `target_heading` field, which will be honored only for waypoints that are skipped, and if the underlying robot 
	 * interface supports the pure-rotation methods.
	 *
	 * Notes on navigation status and event dispatchment:
	 *  - Navigation state may briefly pass by the `IDLE` status between a waypoint is reached and a new navigation command is issued towards the next waypoint.
	 *  - `sendNavigationEndEvent()` will be called only when the last waypoint is reached.
	 *  - Reaching an intermediary waypoint (or skipping it if considered so by the navigator) generates a call to `sendWaypointReachedEvent()` instead.
	 *
	 * \sa Base class CAbstractNavigator, CWaypointsNavigator::navigateWaypoints(), and derived classes.
	 *  \ingroup nav_reactive
	 */
	class NAV_IMPEXP CWaypointsNavigator : public  mrpt::nav::CAbstractNavigator
	{
	public:
		/** The struct for configuring navigation requests to CWaypointsNavigator and derived classes. */
		struct NAV_IMPEXP TNavigationParamsWaypoints : public CAbstractNavigator::TNavigationParams
		{
			/** If not empty, this will prevail over the base class single goal target. 
			  * Semantic is: any of these targets will be good for heading the robot towards them, 
			  * but the priority is for the latest ones in the sequence. */
			std::vector<mrpt::nav::CAbstractNavigator::TargetInfo>  multiple_targets;

			virtual std::string getAsText() const MRPT_OVERRIDE;
			virtual TNavigationParamsBase* clone() const MRPT_OVERRIDE { return new TNavigationParamsWaypoints(*this); }
		protected:
			virtual bool isEqual(const CAbstractNavigator::TNavigationParamsBase& o) const MRPT_OVERRIDE;
		};

		CWaypointsNavigator( CRobot2NavInterface &robot_interface_impl );  //!< ctor
		virtual ~CWaypointsNavigator(); //!< dtor

		// Overriden to call the general navigationStep(), plus waypoint selection logic.
		virtual void navigationStep() MRPT_OVERRIDE;
		virtual void cancel() MRPT_OVERRIDE; //!< Cancel current navegation.

		/** \name Waypoint navigation control API
		  * @{ */

		/** Waypoint navigation request. This immediately cancels any other previous on-going navigation.
		  * \sa CAbstractNavigator::navigate() for single waypoint navigation requests.
		  */
		virtual void navigateWaypoints( const TWaypointSequence & nav_request );

		/** Get a copy of the control structure which describes the progress status of the waypoint navigation. */
		virtual void getWaypointNavStatus(TWaypointStatusSequence & out_nav_status) const;

		/** Get a copy of the control structure which describes the progress status of the waypoint navigation. */
		TWaypointStatusSequence getWaypointNavStatus() const {
			TWaypointStatusSequence nav_status;
			this->getWaypointNavStatus(nav_status);
			return nav_status;
		}
		/** @}*/

		/** Returns `true` if, according to the information gathered at the last navigation step, 
		* there is a free path to the given point; `false` otherwise: if way is blocked or there is missing information, 
		* the point is out of range for the existing PTGs, etc. */
		bool isRelativePointReachable(const mrpt::math::TPoint2D &wp_local_wrt_robot) const;

		struct NAV_IMPEXP TWaypointsNavigatorParams : public mrpt::utils::CLoadableOptions
		{
			double  max_distance_to_allow_skip_waypoint; //!< In meters. <0: unlimited
			int     min_timesteps_confirm_skip_waypoints; //!< How many times shall a future waypoint be seen as reachable to skip to it (Default: 1)
			double  waypoint_angle_tolerance;             //!< [rad] Angular error tolerance for waypoints with an assigned heading (Default: 5 deg)
			double  rel_speed_for_stop_waypoints;         //!< [0,1] Relative speed when aiming at a stop-point waypoint (Default=0.10)
			int     multitarget_look_ahead;               //!< >=0 number of waypoints to forward to the underlying navigation engine, to ease obstacles avoidance when a waypoint is blocked (Default=0 : none).

			virtual void loadFromConfigFile(const mrpt::utils::CConfigFileBase &c, const std::string &s) MRPT_OVERRIDE;
			virtual void saveToConfigFile(mrpt::utils::CConfigFileBase &c, const std::string &s) const MRPT_OVERRIDE;
			TWaypointsNavigatorParams();
		};

		TWaypointsNavigatorParams params_waypoints_navigator;

		virtual void loadConfigFile(const mrpt::utils::CConfigFileBase &c) MRPT_OVERRIDE; // See base class docs!
		virtual void saveConfigFile(mrpt::utils::CConfigFileBase &c) const MRPT_OVERRIDE; // See base class docs!

	protected:
		TWaypointStatusSequence  m_waypoint_nav_status; //!< The latest waypoints navigation command and the up-to-date control status.
		mrpt::synch::CCriticalSectionRecursive m_nav_waypoints_cs;

		/** Implements the way to waypoint is free function in children classes: `true` must be returned 
		  * if, according to the information gathered at the last navigation step, there is a free path to 
		  * the given point; `false` otherwise: if way is blocked or there is missing information, the point is out of range, etc. */
		virtual bool impl_waypoint_is_reachable(const mrpt::math::TPoint2D &wp_local_wrt_robot) const = 0;

		virtual void onStartNewNavigation() MRPT_OVERRIDE;

		virtual bool checkHasReachedTarget(const double targetDist) const MRPT_OVERRIDE;
		virtual void waypoints_navigationStep(); //!< The waypoints-specific part of navigationStep()

		bool waypoints_isAligning() const { return m_is_aligning; }

	private:
		bool m_was_aligning; //!< Whether the last timestep was "is_aligning" in a waypoint with heading
		bool m_is_aligning;
		mrpt::system::TTimeStamp m_last_alignment_cmd;

	};
  }
}

