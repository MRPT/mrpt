/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
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
