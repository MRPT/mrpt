/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2025, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

#pragma once

#ifndef MRPT_NO_WARN_BIG_HDR
MRPT_WARNING(
    "Including <mrpt/maps.h> makes compilation much slower, consider including "
    "only what you need (define MRPT_NO_WARN_BIG_HDR to disable this warning)")
#endif

#include <mrpt/maps/CBeacon.h>
#include <mrpt/maps/CBeaconMap.h>
#include <mrpt/maps/CColouredOctoMap.h>
#include <mrpt/maps/CColouredPointsMap.h>
#include <mrpt/maps/CGasConcentrationGridMap2D.h>
#include <mrpt/maps/CGenericPointsMap.h>
#include <mrpt/maps/CHeightGridMap2D.h>
#include <mrpt/maps/CHeightGridMap2D_MRF.h>
#include <mrpt/maps/CMultiMetricMap.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/maps/COccupancyGridMap3D.h>
#include <mrpt/maps/COctoMap.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/maps/CPointsMapXYZI.h>
#include <mrpt/maps/CPointsMapXYZIRT.h>
#include <mrpt/maps/CRandomFieldGridMap3D.h>
#include <mrpt/maps/CReflectivityGridMap2D.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/maps/CVoxelMap.h>
#include <mrpt/maps/CVoxelMapRGB.h>
#include <mrpt/maps/CWeightedPointsMap.h>
#include <mrpt/maps/CWirelessPowerGridMap2D.h>

//#include <mrpt/maps/PCL_adapters.h>  // NOTE: This file must be included from
// the user code only if he has already include PCL headers.

#include <mrpt/viz/CAngularObservationMesh.h>
#include <mrpt/viz/CPlanarLaserScan.h>
