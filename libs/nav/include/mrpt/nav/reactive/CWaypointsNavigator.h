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
		/** @}*/

		/** Returns `true` if, according to the information gathered at the last navigation step, 
		* there is a free path to the given point; `false` otherwise: if way is blocked or there is missing information, 
		* the point is out of range for the existing PTGs, etc. */
		bool isRelativePointReachable(const mrpt::math::TPoint2D &wp_local_wrt_robot) const;

	protected:
		TWaypointStatusSequence  m_waypoint_nav_status; //!< The latest waypoints navigation command and the up-to-date control status.
		mrpt::synch::CCriticalSectionRecursive m_nav_waypoints_cs;

		double  MAX_DISTANCE_TO_ALLOW_SKIP_WAYPOINT; //!< In meters. <0: unlimited
		int     MIN_TIMESTEPS_CONFIRM_SKIP_WAYPOINTS; //!< How many times shall a future waypoint be seen as reachable to skip to it (Default: 1)

		/** Implements the way to waypoint is free function in children classes: `true` must be returned 
		  * if, according to the information gathered at the last navigation step, there is a free path to 
		  * the given point; `false` otherwise: if way is blocked or there is missing information, the point is out of range, etc. */
		virtual bool impl_waypoint_is_reachable(const mrpt::math::TPoint2D &wp_local_wrt_robot) const = 0;

		/** Loads parameters for waypoints navigation */
		virtual void loadWaypointsParamsConfigFile(const mrpt::utils::CConfigFileBase &cfg, const std::string &sectionName);

		virtual void onStartNewNavigation() MRPT_OVERRIDE;

	};
  }
}

