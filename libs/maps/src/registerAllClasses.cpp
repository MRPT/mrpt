/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "maps-precomp.h"
//
#define MRPT_NO_WARN_BIG_HDR
#include <mrpt/core/initializer.h>
#include <mrpt/maps.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/CObservationRotatingScan.h>

using namespace mrpt::maps;
using namespace mrpt::obs;
using namespace mrpt::opengl;

MRPT_INITIALIZER(registerAllClasses_mrpt_maps)
{
#if !defined(DISABLE_MRPT_AUTO_CLASS_REGISTRATION)
	registerClass(CLASS_ID(CBeacon));
	registerClass(CLASS_ID(CBeaconMap));

	registerClass(CLASS_ID(CPointsMap));
	registerClass(CLASS_ID(CSimplePointsMap));
	registerClass(CLASS_ID(CColouredPointsMap));
	registerClass(CLASS_ID(CWeightedPointsMap));
	registerClass(CLASS_ID(CPointsMapXYZI));
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

	registerClass(CLASS_ID(CAngularObservationMesh));
	registerClass(CLASS_ID(CPlanarLaserScan));

	registerClass(CLASS_ID(CObservationPointCloud));
	registerClass(CLASS_ID(CObservationRotatingScan));

	registerClass(CLASS_ID(CMultiMetricMap));

#endif
}
