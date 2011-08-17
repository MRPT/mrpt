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
#ifndef _motion_planning_utils_H
#define _motion_planning_utils_H

#include <mrpt/reactivenav/CParameterizedTrajectoryGenerator.h>
#include <mrpt/math/CPolygon.h>
#include <mrpt/reactivenav/link_pragmas.h>

namespace mrpt
{
	namespace reactivenav
	{
	/** @name Motion planning utilities
		@{*/
	  
		/** Builds the collision grid for a given list of PTGs. 
		  * The collision grid must be calculated before calling CParameterizedTrajectoryGenerator::CColisionGrid::getTPObstacle
		  *  \param PTGs The list of PTGs to calculate their grids.
		  *  \param robotShape The shape of the robot.
		  *  \param cacheFilesPrefix The prefix of the files where the collision grids will be dumped to speed-up future recalculations.
		  *  \param verbose
		  * \sa CReactiveNavigationSystem
		  *  \ingroup mrpt_reactivenav_grp
		  */
		
		void REACTIVENAV_IMPEXP build_PTG_collision_grids(
			std::vector<CParameterizedTrajectoryGenerator*>	PTGs,
			const mrpt::math::CPolygon						&robotShape,
			const std::string								&cacheFilesPrefix = std::string("ReacNavGrid_"),
			bool											verbose = true
			);

	/** @} */
	}
}


#endif

