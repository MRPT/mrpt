/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "apps-precomp.h"  // Precompiled headers
//
#include <mrpt/apps/registerAllClasses.h>
#include <mrpt/core/initializer.h>
// Deps:
//#include <mrpt/graphslam/registerAllClasses.h> // optional
#include <mrpt/gui/registerAllClasses.h>
//#include <mrpt/hmtslam/registerAllClasses.h> // optional
#include <mrpt/hwdrivers/registerAllClasses.h>
#include <mrpt/slam/registerAllClasses.h>
#include <mrpt/topography/registerAllClasses.h>

MRPT_INITIALIZER(registerAllClasses_mrpt_apps)
{
#if !defined(DISABLE_MRPT_AUTO_CLASS_REGISTRATION)
	// registerClass(CLASS_ID(XXX));
#endif
}

void mrpt::apps::registerAllClasses_mrpt_apps()
{
	::registerAllClasses_mrpt_apps();
	// mrpt::graphslam::registerAllClasses_mrpt_graphslam(); // opt.
	// mrpt::hmtslam::registerAllClasses_mrpt_graphslam();	 // opt.
	mrpt::gui::registerAllClasses_mrpt_gui();
	mrpt::hwdrivers::registerAllClasses_mrpt_hwdrivers();
	mrpt::slam::registerAllClasses_mrpt_slam();
	mrpt::topography::registerAllClasses_mrpt_topography();
}
