/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/system/TParameters.h>

namespace mrpt::nav
{
class CParameterizedTrajectoryGenerator;

/** Stores a candidate movement in TP-Space-based navigation.
 *\sa CReactiveNavigationSystem, CReactiveNavigationSystem3D
 *  \ingroup nav_reactive
 */
struct TCandidateMovementPTG
{
	/** The associated PTG. nullptr if not applicable / undefined. */
	CParameterizedTrajectoryGenerator* PTG{nullptr};
	/** TP-Space movement direction. Within [-2*PI,+2*PI] */
	double direction{0};
	/** TP-Space movement speed, normalized to [0,1]. A negative number means
	 * this candidate movement is unfeasible and must be discarded. */
	double speed{0};
	/** Default to 0, they can be used to reflect a robot starting at a position
	 * not at (0,0). Used in "PTG continuation" */
	double starting_robot_dir{0}, starting_robot_dist{0};

	/** List of properties. May vary for different user implementations of
	 * scores and/or different multi-objective optimizers.
	 * See list of available variable names in docs for
	 * mrpt::nav::CAbstractPTGBasedReactive
	 */
	mrpt::system::TParameters<double> props;

	TCandidateMovementPTG();
};
}  // namespace mrpt::nav
