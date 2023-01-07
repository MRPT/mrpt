/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "slam-precomp.h"  // Precompiled headers
//
#define MRPT_NO_WARN_BIG_HDR
#include <mrpt/core/initializer.h>
#include <mrpt/slam.h>
#include <mrpt/slam/registerAllClasses.h>
// deps:
#include <mrpt/maps/registerAllClasses.h>
#include <mrpt/vision/registerAllClasses.h>

MRPT_INITIALIZER(registerAllClasses_mrpt_slam)
{
	using namespace mrpt::slam;
	using namespace mrpt::maps;

#if !defined(DISABLE_MRPT_AUTO_CLASS_REGISTRATION)
	registerClass(CLASS_ID(CIncrementalMapPartitioner));
	registerClass(CLASS_ID(CMultiMetricMapPDF));
#endif
}

void mrpt::slam::registerAllClasses_mrpt_slam()
{
	::registerAllClasses_mrpt_slam();
	// deps:
	mrpt::vision::registerAllClasses_mrpt_vision();
	mrpt::maps::registerAllClasses_mrpt_maps();
}
