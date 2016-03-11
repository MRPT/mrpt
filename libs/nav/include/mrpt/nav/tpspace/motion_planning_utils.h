/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef _motion_planning_utils_H
#define _motion_planning_utils_H

#include <mrpt/nav/tpspace/CParameterizedTrajectoryGenerator.h>
#include <mrpt/math/CPolygon.h>
#include <mrpt/nav/link_pragmas.h>

namespace mrpt
{
	namespace nav
	{
	/** @name Motion planning utilities
		@{*/
	  
		/** Builds the collision grid for a given PTGs, or load it from a cache file. 
		  * The collision grid must be calculated before calling CParameterizedTrajectoryGenerator::CColisionGrid::getTPObstacle
		  *  \param PTGs The list of PTGs to calculate their grids.
		  *  \param robotShape The shape of the robot.
		  *  \param cacheFilename_col The filename where the collision grids will be dumped to speed-up future recalculations. If it exists upon call, the collision grid will be loaded from here if all PTG parameters match. Example: "PTG_%03d.dat.gz".
		  *  \param cacheFilename_cle The filename where the clearance grids will be dumped to speed-up future recalculations. If it exists upon call, the clearance grid will be loaded from here if all PTG parameters match. Example: "PTG_%03d.dat.gz".
		  *  \param max_clearance is given in "number of cells" and not in meters or centimeters
		  * \sa CReactiveNavigationSystem
		  *  \ingroup nav_tpspace
		  */
		void NAV_IMPEXP build_PTG_collision_grids(
			CParameterizedTrajectoryGenerator * PTG,
			const mrpt::math::CPolygon        & robotShape,
			const std::string                 & cacheFilename_col,
			const std::string                 & cacheFilename_cle,
			const unsigned int 					max_Clearance,
			const bool                          verbose = true
			);

	/** @} */
	}
}


#endif

