/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
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

#include <mrpt/maps/CBeacon.h>
#include <mrpt/maps/CBeaconMap.h>
#include <mrpt/maps/CColouredPointsMap.h>
#include <mrpt/maps/CGasConcentrationGridMap2D.h>
#include <mrpt/maps/CWirelessPowerGridMap2D.h>
#include <mrpt/maps/CRandomFieldGridMap3D.h>
#include <mrpt/maps/CHeightGridMap2D.h>
#include <mrpt/maps/CHeightGridMap2D_MRF.h>
#include <mrpt/maps/CReflectivityGridMap2D.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/maps/CWeightedPointsMap.h>
#include <mrpt/maps/COctoMap.h>
#include <mrpt/maps/CColouredOctoMap.h>

//#include <mrpt/maps/PCL_adapters.h>  // NOTE: This file must be included from the user
                                       // code only if he has already #include'd PCL headers.

#include <mrpt/opengl/CAngularObservationMesh.h>
#include <mrpt/opengl/CPlanarLaserScan.h>

#endif
