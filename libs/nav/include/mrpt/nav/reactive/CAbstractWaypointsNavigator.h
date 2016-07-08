/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/nav/reactive/CAbstractNavigator.h>

namespace mrpt
{
  namespace nav
  {
	/** Base virtual class for reactive/planned navigation system capable of following a list of waypoints. See derived classes.
	 * \sa base class CAbstractNavigator, CAbstractWaypointsNavigator::navigateWaypoints(), and derived classes.
	 *  \ingroup nav_reactive
	 */
	class NAV_IMPEXP CAbstractWaypointsNavigator : public  mrpt::nav::CAbstractNavigator
	{
	public:
		CAbstractWaypointsNavigator( CRobot2NavInterface &robot_interface_impl );  //!< ctor
		virtual ~CAbstractWaypointsNavigator(); //!< dtor

		/** The struct for requesting waypoint navigation requests. Used in CAbstractWaypointsNavigator::navigateWaypoints() */
		struct NAV_IMPEXP TWaypointNavigationCommand
		{
			MRPT_TODO("cont");
			// mrpt::math::TPose2D target;  //!< Coordinates of desired target location. Heading may be ignored by some reactive implementations.
			float               targetAllowedDistance;    //!< (Default=0.5 meters) Allowed distance to target in order to end the navigation.

			TWaypointNavigationCommand(); //!< Ctor with default values
			virtual std::string getAsText() const; //!< Gets navigation params as a human-readable format
		};

		/** \name Waypoint navigation control API
		  * @{ */

		/** Waypoint navigation request.
		  * \sa CAbstractNavigator::navigate() for single waypoint navigation requests.
		  */
		virtual void  navigate( const TWaypointNavigationCommand &params );

		/** @}*/


	protected:

	};
  }
}

