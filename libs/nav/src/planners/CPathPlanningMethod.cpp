/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2015, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "nav-precomp.h"   // Precompiled headers 

#include <mrpt/nav/planners/CPathPlanningMethod.h>

using namespace mrpt::slam;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::nav;

/*---------------------------------------------------------------
						Constructor
  ---------------------------------------------------------------*/
CPathPlanningMethod::CPathPlanningMethod()
	:	occupancyThreshold(0.7f),
		minStepInReturnedPath(0.4f)
{
}


/*---------------------------------------------------------------
  ---------------------------------------------------------------*/
