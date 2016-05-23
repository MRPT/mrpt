/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "nav-precomp.h" // Precomp header

#include <mrpt/nav/tpspace/CParameterizedTrajectoryGenerator.h>

using namespace mrpt::nav;

std::string CParameterizedTrajectoryGenerator::OUTPUT_DEBUG_PATH_PREFIX = "./reactivenav.logs";


CParameterizedTrajectoryGenerator::CParameterizedTrajectoryGenerator(const mrpt::utils::TParameters<double> &params)
{
	refDistance        = params["ref_distance"];
	m_alphaValuesCount = params["num_paths"];
}

