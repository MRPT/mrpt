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
	/** \defgroup nav_tpspace TP-Space and PTG classes
	  * \ingroup mrpt_nav_grp
	  */

	/** Clearance information for one particular PTG and one set of obstacles. 
		* Usage: declare an object of this type (it will be initialized to "empty"), then 
		* repeatedly call CParameterizedTrajectoryGenerator::updateClearance() for each 2D obstacle point. */
	class NAV_IMPEXP ClearanceDiagram
	{
	public:
		/** Container: [path_k][TPS_distance] => normalized_clearance_for_exactly_that_robot_pose  */
		std::vector<std::map<double,double> > raw_clearances;

		ClearanceDiagram(); //!< default ctor
		void clear(); //!< Reset to default, empty state
		double getClearance(uint16_t k, double TPS_query_distance) const; //!< Interpolate raw_clearances to get a smooth clearance function over paths.
		void renderAs3DObject(mrpt::opengl::CMesh &mesh, double min_x, double max_x, double min_y, double max_y, double cell_res) const;
	};


}
}
