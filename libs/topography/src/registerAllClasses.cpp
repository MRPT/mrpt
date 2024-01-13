/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "topography-precomp.h"
//
#include <mrpt/core/initializer.h>
#include <mrpt/topography.h>
#include <mrpt/topography/registerAllClasses.h>
// deps:
#include <mrpt/obs/registerAllClasses.h>

MRPT_INITIALIZER(registerAllClasses_mrpt_topography)
{
#if !defined(DISABLE_MRPT_AUTO_CLASS_REGISTRATION)
//	registerClass( CLASS_ID( XXXX ) );
#endif
}

void mrpt::topography::registerAllClasses_mrpt_topography()
{
	::registerAllClasses_mrpt_topography();
	// deps:
	mrpt::obs::registerAllClasses_mrpt_obs();
}
