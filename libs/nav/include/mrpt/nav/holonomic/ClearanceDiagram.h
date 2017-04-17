/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#pragma once

#include <vector>
#include <map>
#include <mrpt/utils/mrpt_stdint.h>    // compiler-independent version of "stdint.h"
#include <mrpt/nav/link_pragmas.h>

namespace mrpt { namespace opengl { class CMesh; } }

namespace mrpt
{
namespace nav
{
	/** Clearance information for one particular PTG and one set of obstacles. 
		* Usage: 
		* - Declare an object of this type (it will be initialized to "empty"), 
		* - Call CParameterizedTrajectoryGenerator::initClearanceDiagram()
		* - Repeatedly call CParameterizedTrajectoryGenerator::updateClearance() for each 2D obstacle point. 
		*
		*  \ingroup nav_tpspace
		*/
	class NAV_IMPEXP ClearanceDiagram
	{
	public:
		/** Container: [decimated_path_k][TPS_distance] => normalized_clearance_for_exactly_that_robot_pose  */
		std::vector<std::map<double,double> > raw_clearances;

		ClearanceDiagram(); //!< default ctor
		void clear(); //!< Reset to default, empty state
		/** Gets the clearance for path `k` and distance `TPS_query_distance` in one of two modes: 
		  * - [integrate_over_path=false] clearance from that specific spot, or 
		  * - [integrate_over_path=true] average clearance over the path from the origin to that specific spot.
		  */
		double getClearance(uint16_t k, double TPS_query_distance, bool integrate_over_path) const;
		void renderAs3DObject(mrpt::opengl::CMesh &mesh, double min_x, double max_x, double min_y, double max_y, double cell_res, bool integrate_over_path) const;
	};


}
}
