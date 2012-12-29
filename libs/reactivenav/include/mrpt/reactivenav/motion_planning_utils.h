/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                                 |
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

