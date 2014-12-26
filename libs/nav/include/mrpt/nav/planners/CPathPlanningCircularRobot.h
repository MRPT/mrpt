/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2015, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CPathPlanningCircularRobot_H
#define CPathPlanningCircularRobot_H

#include <mrpt/nav/planners/CPathPlanningMethod.h>
#include <mrpt/nav/link_pragmas.h>

namespace mrpt
{
namespace nav
{
	/** An implementation of CPathPlanningMethod
	 * \sa CPathPlanningMethod \ingroup mrpt_nav_grp
	 */
	class NAV_IMPEXP CPathPlanningCircularRobot : public CPathPlanningMethod
	{
	public:
		/** Default constructor
		  */
		CPathPlanningCircularRobot();

		/** Destructor
		  */
        virtual ~CPathPlanningCircularRobot()
		{
		}

		/** The aproximate robot radius used in the planification. Default is 0.35m
		  */
		float	robotRadius;

		/** This method compute the optimal path for a circular robot, in the given
		  *   occupancy grid map, from the origin location to a target point.
		  * The options and additional parameters to this method can be set with
		  *   member configuration variables.
		  *
		  * \param theMap	[IN] The occupancy gridmap used to the planning.
		  * \param origin	[IN] The starting pose of the robot, in coordinates of "map".
		  * \param target	[IN] The desired target pose for the robot, in coordinates of "map".
		  * \param path		[OUT] The found path, in global coordinates relative to "map".
		  * \param notFount	[OUT] Will be true if no path has been found.
		  * \param maxSearchPathLength [IN] The maximum path length to search for, in meters (-1 = no limit)
		  *
		  * \sa robotRadius
		  *
		  * \exception std::exception On any error
		  */
		void  computePath(
				const mrpt::slam::COccupancyGridMap2D	&theMap,
				const mrpt::poses::CPose2D				&origin,
				const mrpt::poses::CPose2D				&target,
				std::deque<mrpt::math::TPoint2D>	&path,
				bool						&notFound,
				float						maxSearchPathLength = -1
				) const;

	};

	} // End of namespace
} // End of namespace

#endif
