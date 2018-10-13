/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/poses/poses_frwds.h>
// Forward declarations for the library "mrpt-obs"
namespace mrpt
{
namespace obs
{
class CObservation;
class CSensoryFrame;
class CObservation2DRangeScan;
class CObservation3DRangeScan;
class CObservationVelodyneScan;
class CObservationRange;
class CObservationBeaconRanges;
class CObservationBearingRange;
class CObservationStereoImages;
class CObservationGPS;
}  // namespace obs
namespace maps
{
class CMetricMap;
class CPointsMap;
class CSimplePointsMap;
class CSimpleMap;
}  // namespace maps
}  // namespace mrpt
