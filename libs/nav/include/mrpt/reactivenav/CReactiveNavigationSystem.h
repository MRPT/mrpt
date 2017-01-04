/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/utils/mrpt_macros.h>

MRPT_WARNING("Deprecated header: use  <mrpt/nav/reactive/CReactiveNavigationSystem.h> instead")

#include <mrpt/nav/reactive/CReactiveNavigationSystem.h>

// MRPT <1.3.0 backwards compatibility 
namespace mrpt {
	namespace  reactivenav = mrpt::nav;
}
