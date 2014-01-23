/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef _mrpt_maps_H
#define _mrpt_maps_H

#include <mrpt/config.h>

// Only really include all headers if we come from a user program (anything
//  not defining mrpt_*_EXPORTS) or MRPT is being built with precompiled headers.
#if !defined(mrpt_maps_EXPORTS) || MRPT_ENABLE_PRECOMPILED_HDRS || defined(MRPT_ALWAYS_INCLUDE_ALL_HEADERS)

#include <mrpt/slam/CBeacon.h>
#include <mrpt/slam/CBeaconMap.h>
#include <mrpt/slam/CColouredPointsMap.h>
#include <mrpt/slam/CGasConcentrationGridMap2D.h>
#include <mrpt/slam/CWirelessPowerGridMap2D.h>
#include <mrpt/slam/CHeightGridMap2D.h>
#include <mrpt/slam/CReflectivityGridMap2D.h>
#include <mrpt/slam/COccupancyGridMap2D.h>
#include <mrpt/slam/CPointsMap.h>
#include <mrpt/slam/CSimplePointsMap.h>
#include <mrpt/slam/CWeightedPointsMap.h>
#include <mrpt/slam/COctoMap.h>
#include <mrpt/slam/CColouredOctoMap.h>

//#include <mrpt/slam/PCL_adapters.h>  // NOTE: This file must be included from the user
                                       // code only if he has already #include'd PCL headers.

#include <mrpt/opengl/CAngularObservationMesh.h>
#include <mrpt/opengl/CPlanarLaserScan.h>

#endif // end precomp.headers

#endif
