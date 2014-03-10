/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef RRTAlgorithm_H
#define RRTAlgorithm_H

#include <mrpt/utils.h>

namespace mrpt
{
  namespace hybridnav
  {

        /** This class implement an header-only RRT algorithm for hybrid navigation
         *
         *  <b>Usage:</b><br>
         *		- write me
         *
         *
         *  <b>About the algorithm:</b><br>
         *
         *
         * <b>Changes history</b>
         *      - 06/MAR/2014: Creation (MB)
         *  \ingroup mrpt_hybridnav_grp
         */

		template <class  POSE, class MOTIONS>   //this is to be used for any pose and any motion
		struct Algorithm_RRT
		{
			void plan(const POSE &start_pose, const POSE &goal_pose)
			{
			// Like in the RRT pseudocode,
			// try  to be as generic as posible, assuming the existence of methods to be implemented by the “user” in derived classes
			// e.g.:
			this->addEdge(start_pose, start_pose);
			};

			mrpt::poses::CPose2D start_pose;
			mrpt::poses::CPose2D goal_pose;
		};
  }
}
#endif // RRTAlgorithm_H
