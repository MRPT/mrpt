/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef PlannerSimple2D_H
#define PlannerSimple2D_H

#include <mrpt/nav/link_pragmas.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/poses/CPoint2D.h>

namespace mrpt
{
namespace nav
{
	/** \addtogroup nav_planners Path planning
	  * \ingroup mrpt_nav_grp
	  * @{ */
	  
	/** Searches for collision-free path in 2D occupancy grids for holonomic circular robots. 
	 *  The implementation first enlargest obstacles with robot radius, then applies a 
	 *  wavefront algorithm to find the shortest free path between origin and target 2D points.
	 *
	 * Notice that this simple planner does not take into account robot kinematic constraints.
	 */
	class NAV_IMPEXP PlannerSimple2D
	{
	public:
		PlannerSimple2D();  //!< Default constructor
		virtual ~PlannerSimple2D() //!< Destructor
		{
		}

		/** The maximum occupancy probability to consider a cell as an obstacle, default=0.5  */
		float	occupancyThreshold;
		
		/** The minimum distance between points in the returned found path (default=0.4); Notice
		  *  that full grid resolution is used in path finding, this is only a way to reduce the
		  *  amount of redundant information to be returned.
		  */
		float	minStepInReturnedPath;

		float	robotRadius;  //!< The aproximate robot radius used in the planification. Default is 0.35m

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
				const mrpt::maps::COccupancyGridMap2D	&theMap,
				const mrpt::poses::CPose2D				&origin,
				const mrpt::poses::CPose2D				&target,
				std::deque<mrpt::math::TPoint2D>	&path,
				bool						&notFound,
				float						maxSearchPathLength = -1
				) const;

	};

	  /** @} */
	} // End of namespace
} // End of namespace

#endif
