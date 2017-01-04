/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef mrpt_obs_H
#define mrpt_obs_H

#ifndef MRPT_NO_WARN_BIG_HDR
#include <mrpt/utils/core_defs.h>
MRPT_WARNING("Including <mrpt/obs.h> makes compilation much slower, consider including only what you need (define MRPT_NO_WARN_BIG_HDR to disable this warning)")
#endif

#include <mrpt/config.h>

// Observations:
#include <mrpt/obs/CObservation.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation2DRangeScanWithUncertainty.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationRGBD360.h>
#include <mrpt/obs/CObservationRange.h>
#include <mrpt/obs/CObservationImage.h>
// #include <mrpt/obs/CObservationVisualLandmarks.h>  // This one is in mrpt-core
#include <mrpt/obs/CObservationStereoImages.h>
#include <mrpt/obs/CObservationStereoImagesFeatures.h>
#include <mrpt/obs/CObservationBeaconRanges.h>
#include <mrpt/obs/CObservation6DFeatures.h>
#include <mrpt/obs/CObservationRobotPose.h>
#include <mrpt/obs/CObservationGasSensors.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/obs/CObservationBatteryState.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CObservationBearingRange.h>
#include <mrpt/obs/CObservationComment.h>
#include <mrpt/obs/CObservationReflectivity.h>
#include <mrpt/obs/CObservationWirelessPower.h>
#include <mrpt/obs/CObservationRFID.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/obs/CObservationWindSensor.h>
#include <mrpt/obs/CObservationCANBusJ1939.h>
#include <mrpt/obs/CObservationRawDAQ.h>
#include <mrpt/obs/CObservationSkeleton.h>
#include <mrpt/obs/CObservationVelodyneScan.h>

// Observations:
#include <mrpt/obs/CAction.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/obs/CActionRobotMovement3D.h>


// Others:
#include <mrpt/obs/CRawlog.h>
#include <mrpt/obs/carmen_log_tools.h>

// Very basic classes for maps:
#include <mrpt/maps/CMetricMap.h>
#include <mrpt/maps/CSimpleMap.h>


#endif
