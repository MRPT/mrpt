/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "nav-precomp.h" // Precomp header

#include <mrpt/nav/reactive/TCandidateMovementPTG.h>

using namespace mrpt::nav;

TCandidateMovementPTG::TCandidateMovementPTG() : 
	PTG(nullptr), 
	direction(0), speed(0), 
	starting_robot_dir(0), starting_robot_dist(0) 
{
}

