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
#ifndef CPathPlanningCircularRobot_H
#define CPathPlanningCircularRobot_H

#include <mrpt/slam/CPathPlanningMethod.h>

#include <mrpt/slam/link_pragmas.h>


namespace mrpt
{
namespace slam
{
	/** An implementation of CPathPlanningMethod
	 * \sa CPathPlanningMethod \ingroup mrpt_slam_grp
	 */
	class SLAM_IMPEXP CPathPlanningCircularRobot : public CPathPlanningMethod
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
				const COccupancyGridMap2D	&theMap,
				const CPose2D				&origin,
				const CPose2D				&target,
				std::deque<math::TPoint2D>	&path,
				bool						&notFound,
				float						maxSearchPathLength = -1
				) const;

	};

	} // End of namespace
} // End of namespace

#endif
