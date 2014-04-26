/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
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
#include <mrpt/slam/CObservation.h>
#include <mrpt/slam/CObservation2DRangeScan.h>
#include <mrpt/slam/CObservation3DRangeScan.h>
#include <mrpt/slam/CObservationRGBD360.h>
#include <mrpt/slam/CObservationRange.h>
#include <mrpt/slam/CObservationImage.h>
// #include <mrpt/slam/CObservationVisualLandmarks.h>  // This one is in mrpt-core
#include <mrpt/slam/CObservationStereoImages.h>
#include <mrpt/slam/CObservationStereoImagesFeatures.h>
#include <mrpt/slam/CObservationBeaconRanges.h>
#include <mrpt/slam/CObservationGasSensors.h>
#include <mrpt/slam/CObservationGPS.h>
#include <mrpt/slam/CObservationBatteryState.h>
#include <mrpt/slam/CObservationIMU.h>
#include <mrpt/slam/CObservationOdometry.h>
#include <mrpt/slam/CObservationBearingRange.h>
#include <mrpt/slam/CObservationComment.h>
#include <mrpt/slam/CObservationReflectivity.h>
#include <mrpt/slam/CObservationWirelessPower.h>
#include <mrpt/slam/CObservationRFID.h>
#include <mrpt/slam/CSensoryFrame.h>
#include <mrpt/slam/CObservationWindSensor.h>
#include <mrpt/slam/CObservationCANBusJ1939.h>
#include <mrpt/slam/CObservationRawDAQ.h>


// Observations:
#include <mrpt/slam/CAction.h>
#include <mrpt/slam/CActionCollection.h>
#include <mrpt/slam/CActionRobotMovement2D.h>
#include <mrpt/slam/CActionRobotMovement3D.h>


// Others:
#include <mrpt/slam/CRawlog.h>
#include <mrpt/slam/carmen_log_tools.h>

// Very basic classes for maps:
#include <mrpt/slam/CMetricMap.h>
#include <mrpt/slam/CSimpleMap.h>


#endif
