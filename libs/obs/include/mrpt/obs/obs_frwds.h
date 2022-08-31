/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
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
