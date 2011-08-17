/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */
#ifndef CPathPlanningMethod_H
#define CPathPlanningMethod_H

#include <mrpt/utils/CDebugOutputCapable.h>
#include <mrpt/slam/COccupancyGridMap2D.h>
#include <mrpt/poses/CPoint2D.h>

#include <mrpt/slam/link_pragmas.h>

namespace mrpt
{
namespace slam
{
	/** A virtual base class for computing the optimal path for a robot
	 *	  from a origin location to a target point. See derived classes for
	 *    implementations.
     *
	 * \sa CDebugOutputCapable  \ingroup mrpt_slam_grp
	 */
	class SLAM_IMPEXP CPathPlanningMethod : public mrpt::utils::CDebugOutputCapable
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
				const COccupancyGridMap2D	&theMap,
				const CPose2D				&origin,
				const CPose2D				&target,
				std::deque<math::TPoint2D>	&path,
				bool						&notFound,
				float						maxSearchPathLength = -1
				) const = 0;

	};

	} // End of namespace
} // End of namespace

#endif
