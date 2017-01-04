/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/poses/poses_frwds.h>
// Forward declarations for the library "mrpt-obs"
namespace mrpt
{
	namespace obs
	{
		class CObservation; struct CObservationPtr;
		class CSensoryFrame; struct CSensoryFramePtr;
		class CObservation2DRangeScan;
		class CObservation3DRangeScan;
		class CObservationVelodyneScan;
		class CObservationRange;
		class CObservationBeaconRanges;
		class CObservationBearingRange;
		class CObservationStereoImages;
		class CObservationGPS;
	}
	namespace maps
	{
		class CMetricMap; struct CMetricMapPtr;
		class CPointsMap;
		class CSimplePointsMap;
		class CSimpleMap;
	}
}
