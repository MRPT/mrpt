/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2015, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef mrpt_slam_H
#define mrpt_slam_H

#ifndef MRPT_NO_WARN_BIG_HDR
#include <mrpt/utils/core_defs.h>
MRPT_WARNING("Including <mrpt/slam.h> makes compilation much slower, consider including only what you need (define MRPT_NO_WARN_BIG_HDR to disable this warning)")
#endif

#include <mrpt/config.h>

// Maps:
#include <mrpt/maps/CMetricMap.h>
#include <mrpt/slam/CPointsMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/maps/CColouredPointsMap.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/maps/CMultiMetricMap.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/maps/CLandmarksMap.h>
#include <mrpt/slam/CLandmark.h>
#include <mrpt/maps/CGasConcentrationGridMap2D.h>

#include <mrpt/maps/CWirelessPowerGridMap2D.h>

#include <mrpt/maps/CBeaconMap.h>

// Map Building algorithms:
#include <mrpt/slam/CMetricMapBuilderICP.h>
#include <mrpt/slam/CMetricMapBuilderRBPF.h>
#include <mrpt/slam/CRangeBearingKFSLAM.h>
#include <mrpt/slam/CRangeBearingKFSLAM2D.h>

// Observations:
#include <mrpt/obs/CObservation.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationRange.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/obs/CObservationVisualLandmarks.h>
#include <mrpt/obs/CObservationStereoImages.h>
#include <mrpt/obs/CObservationBeaconRanges.h>
#include <mrpt/obs/CObservationGasSensors.h>
#include <mrpt/obs/CObservationWirelessPower.h>
#include <mrpt/obs/CObservationRFID.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/obs/CObservationBatteryState.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CObservationBearingRange.h>
#include <mrpt/obs/CObservationComment.h>

#include <mrpt/slam/observations_overlap.h>

#include <mrpt/obs/CSensoryFrame.h>

// Actions:
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/obs/CActionRobotMovement3D.h>


// Algorithms:
#include <mrpt/slam/CMonteCarloLocalization2D.h>
#include <mrpt/slam/CMonteCarloLocalization3D.h>

#include <mrpt/slam/CICP.h>
#include <mrpt/slam/CGridMapAligner.h>

#include <mrpt/slam/CIncrementalMapPartitioner.h>
#include <mrpt/slam/CRejectionSamplingRangeOnlyLocalization.h>

#include <mrpt/slam/data_association.h>

// PDFs:
#include <mrpt/slam/CMultiMetricMapPDF.h>

// Others:
#include <mrpt/slam/CRawlog.h>

#include <mrpt/slam/COccupancyGridMapFeatureExtractor.h>

#endif
