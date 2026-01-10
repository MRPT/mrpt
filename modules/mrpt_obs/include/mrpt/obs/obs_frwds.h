/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/
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
class CObservation2DRangeScanWithUncertainty;
class CObservation3DRangeScan;
class CObservation3DScene;
class CObservation6DFeatures;
class CObservationBatteryState;
class CObservationBeaconRanges;
class CObservationBearingRange;
class CObservationCANBusJ1939;
class CObservationComment;
class CObservationGPS;
class CObservationGasSensors;
class CObservationIMU;
class CObservationImage;
class CObservationOdometry;
class CObservationRFID;
class CObservationRGBD360;
class CObservationRange;
class CObservationRawDAQ;
class CObservationReflectivity;
class CObservationRobotPose;
class CObservationRotatingScan;
class CObservationSkeleton;
class CObservationStereoImages;
class CObservationStereoImagesFeatures;
class CObservationVelodyneScan;
class CObservationWindSensor;
class CObservationWirelessPower;

}  // namespace obs
namespace maps
{
class CMetricMap;
class CPointsMap;
class CSimplePointsMap;
class CSimpleMap;
}  // namespace maps
}  // namespace mrpt
