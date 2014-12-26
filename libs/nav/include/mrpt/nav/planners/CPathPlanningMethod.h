/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2015, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CPathPlanningMethod_H
#define CPathPlanningMethod_H

#include <mrpt/utils/CDebugOutputCapable.h>
#include <mrpt/slam/COccupancyGridMap2D.h>
#include <mrpt/poses/CPoint2D.h>

#include <mrpt/nav/link_pragmas.h>

namespace mrpt
{
namespace nav
{
	/** A virtual base class for computing the optimal path for a robot
	 *	  from a origin location to a target point. See derived classes for
	 *    implementations.
     *
	 * \sa CDebugOutputCapable  \ingroup mrpt_nav_grp
	 */
	class NAV_IMPEXP CPathPlanningMethod : public mrpt::utils::CDebugOutputCapable
	{
	public:
		/** Default constructor
		  */
		CPathPlanningMethod();

		/** Destructor
		 */
		virtual ~CPathPlanningMethod()
		{
		}

		/** The maximum occupancy probability to consider a cell as an obstacle, default=0.5
		  */
		float	occupancyThreshold;

		/** The minimum distance between points in the returned found path (default=0.4); Notice
		  *  that full grid resolution is used in path finding, this is only a way to reduce the
		  *  amount of redundant information to be returned.
		  */
		float	minStepInReturnedPath;

		/** This method compute the optimal path for a circular robot, in the given
		  *   occupancy grid map, from the origin location to a target point.
		  * The options and additional parameters to this method can be set with
		  *   member configuration variables.
		  *
		  * \param map		[IN] The occupancy gridmap used to the planning.
		  * \param origin	[IN] The starting pose of the robot, in coordinates of "map".
		  * \param target	[IN] The desired target pose for the robot, in coordinates of "map".
		  * \param path		[OUT] The found path, in global coordinates relative to "map".
		  * \param notFound	[OUT] Will be true if no path has been found.
		  * \param maxSearchPathLength [IN] The maximum path length to search for, in meters (-1 = no limit)
		  *
		  * \exception std::exception On any error
		  */
		virtual void  computePath(
				const mrpt::slam::COccupancyGridMap2D	&theMap,
				const mrpt::poses::CPose2D				&origin,
				const mrpt::poses::CPose2D				&target,
				std::deque<mrpt::math::TPoint2D>	&path,
				bool						&notFound,
				float						maxSearchPathLength = -1
				) const = 0;

	};

	} // End of namespace
} // End of namespace

#endif
