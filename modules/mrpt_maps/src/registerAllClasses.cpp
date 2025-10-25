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

#include "maps-precomp.h"
//
#define MRPT_NO_WARN_BIG_HDR
#include <mrpt/core/initializer.h>
#include <mrpt/maps.h>
#include <mrpt/maps/registerAllClasses.h>
#include <mrpt/obs/CObservationPointCloud.h>
// deps:
#include <mrpt/graphs/registerAllClasses.h>
#include <mrpt/obs/registerAllClasses.h>

MRPT_INITIALIZER(registerAllClasses_mrpt_maps)
{
  using namespace mrpt::maps;
  using namespace mrpt::obs;
  using namespace mrpt::viz;

#if !defined(DISABLE_MRPT_AUTO_CLASS_REGISTRATION)
  registerClass(CLASS_ID(CBeacon));
  registerClass(CLASS_ID(CBeaconMap));

  registerClass(CLASS_ID(CPointsMap));
  registerClass(CLASS_ID(CSimplePointsMap));
  registerClass(CLASS_ID(CColouredPointsMap));
  registerClass(CLASS_ID(CWeightedPointsMap));
  registerClass(CLASS_ID(CPointsMapXYZI));
  registerClass(CLASS_ID(CPointsMapXYZIRT));
  registerClass(CLASS_ID(CGenericPointsMap));
  registerClass(CLASS_ID(COccupancyGridMap2D));
  registerClass(CLASS_ID(COccupancyGridMap3D));
  registerClass(CLASS_ID(CGasConcentrationGridMap2D));
  registerClass(CLASS_ID(CWirelessPowerGridMap2D));
  registerClass(CLASS_ID(CRandomFieldGridMap3D));
  registerClass(CLASS_ID(CHeightGridMap2D));
  registerClass(CLASS_ID(CHeightGridMap2D_MRF));
  registerClass(CLASS_ID(CReflectivityGridMap2D));

  registerClass(CLASS_ID(COctoMap));
  registerClass(CLASS_ID(CColouredOctoMap));

  registerClass(CLASS_ID(CVoxelMap));
  registerClass(CLASS_ID(CVoxelMapRGB));

  registerClass(CLASS_ID(CAngularObservationMesh));
  registerClass(CLASS_ID(CPlanarLaserScan));

  registerClass(CLASS_ID(CObservationPointCloud));

  registerClass(CLASS_ID(CMultiMetricMap));

#endif
}

void mrpt::maps::registerAllClasses_mrpt_maps()
{
  ::registerAllClasses_mrpt_maps();
  // deps:
  mrpt::obs::registerAllClasses_mrpt_obs();
  mrpt::graphs::registerAllClasses_mrpt_graphs();
}
