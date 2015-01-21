/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2015, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/nav/planners/TMoveTree.h>

#include <mrpt/nav/link_pragmas.h>

namespace mrpt
{
	namespace nav
	{
		/** TP Space-based RRT path planning for SE(2) (planar) robots.
		*
		*  - Usage:
		*   - write me
		*
		*  - About the algorithm:
		*    - write me
		*
		*  - Changes history:
		*    - 06/MAR/2014: Creation (MB)
		*    - 06/JAN/2015: Refactoring (JLBC)
		*
		*  \ingroup mrpt_nav_grp
		*
		*  \todo Factorize into more generic path planner classes!  //template <class  POSE, class MOTIONS>...
		*/
		class NAV_IMPEXP PlannerRRT_SE2_TPS
		{
		public:
			struct NAV_IMPEXP TAlgorithmParams
			{
				double goalBias;  //!< Probabily of picking the goal as random target (in [0,1], default=0.05)
				double maxLength; //!< Max length of each edge path (in meters, default=1.0)

				TAlgorithmParams() :
					goalBias(0.05),
					maxLength(1.0)
				{
				}
			};
			TAlgorithmParams params; //!< Parameters specific to this path solver algorithm

			struct NAV_IMPEXP TPlannerInput
			{
				mrpt::math::TPose2D  start_pose;
				mrpt::math::TPose2D  goal_pose;
			};

			struct NAV_IMPEXP TPlannerResult
			{
				TMoveTreeSE2_TP move_tree;  //!< The computed motion tree
			};

			PlannerRRT_SE2_TPS();



		};
	}
}
