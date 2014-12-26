/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2015, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef _mrpt_maps_H
#define _mrpt_maps_H

#ifndef MRPT_NO_WARN_BIG_HDR
#include <mrpt/utils/core_defs.h>
MRPT_WARNING("Including <mrpt/maps.h> makes compilation much slower, consider including only what you need (define MRPT_NO_WARN_BIG_HDR to disable this warning)")
#endif

#include <mrpt/config.h>

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

#endif
