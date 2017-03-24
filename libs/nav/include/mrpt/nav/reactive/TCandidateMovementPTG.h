/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/utils/TParameters.h>
#include <mrpt/nav/link_pragmas.h>

namespace mrpt
{
	namespace nav
	{
		class CParameterizedTrajectoryGenerator;

		/** Stores a candidate movement in TP-Space-based navigation.
		*\sa CReactiveNavigationSystem, CReactiveNavigationSystem3D
		*  \ingroup nav_reactive
		*/
		struct NAV_IMPEXP TCandidateMovementPTG
		{
			CParameterizedTrajectoryGenerator *PTG;	//!< The associated PTG. nullptr if not applicable / undefined.
			double direction;    //!< TP-Space movement direction. Within [-2*PI,+2*PI]
			double speed;        //!< TP-Space movement speed, normalized to [0,1]. A negative number means this candidate movement is unfeasible and must be discarded.
			double starting_robot_dir, starting_robot_dist; //!< Default to 0, they can be used to reflect a robot starting at a position not at (0,0). Used in "PTG continuation"

			/** List of properties. May vary for different user implementations of scores and/or different multi-objective optimizers.
			* See list of available variable names in docs for mrpt::nav::CAbstractPTGBasedReactive
			*/
			mrpt::utils::TParameters<double>  props;

			TCandidateMovementPTG();
		};

	}
}

