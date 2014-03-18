/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef __mrpt_reactivenav_H
#define __mrpt_reactivenav_H

// Only really include all headers if we come from a user program (anything
//  not defining mrpt_*_EXPORTS) or MRPT is being built with precompiled headers.
#if !defined(mrpt_reactivenav_EXPORTS) || MRPT_ENABLE_PRECOMPILED_HDRS || defined(MRPT_ALWAYS_INCLUDE_ALL_HEADERS)

#include <mrpt/reactivenav/CReactiveNavigationSystem.h>
#include <mrpt/reactivenav/CReactiveNavigationSystem3D.h>
#include <mrpt/reactivenav/CAbstractReactiveNavigationSystem.h>
#include <mrpt/reactivenav/CPRRTNavigator.h>
#include <mrpt/reactivenav/motion_planning_utils.h>
#include <mrpt/reactivenav/CPTG1.h>
#include <mrpt/reactivenav/CPTG2.h>
#include <mrpt/reactivenav/CPTG3.h>
#include <mrpt/reactivenav/CPTG4.h>
#include <mrpt/reactivenav/CPTG5.h>
#include <mrpt/reactivenav/CPTG6.h>
#include <mrpt/reactivenav/CPTG7.h>

#endif // end precomp.headers

#endif
