/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
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

		void REACTIVENAV_IMPEXP build_PTG_collision_grid3D(
			CParameterizedTrajectoryGenerator*				ptg,
			const mrpt::math::CPolygon						&robotShape,
			const unsigned int								level,
			const unsigned int								num_ptg,
			bool											verbose = true
			);

	/** @} */
	}
}


#endif

