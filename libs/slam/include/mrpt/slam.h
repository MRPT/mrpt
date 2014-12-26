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
#include <mrpt/slam/CMetricMap.h>
#include <mrpt/slam/CPointsMap.h>
#include <mrpt/slam/CSimplePointsMap.h>
#include <mrpt/slam/CColouredPointsMap.h>
#include <mrpt/slam/COccupancyGridMap2D.h>
#include <mrpt/slam/CMultiMetricMap.h>
#include <mrpt/slam/CSimpleMap.h>
#include <mrpt/slam/CLandmarksMap.h>
#include <mrpt/slam/CLandmark.h>
#include <mrpt/slam/CGasConcentrationGridMap2D.h>

#include <mrpt/slam/CWirelessPowerGridMap2D.h>

#include <mrpt/slam/CBeaconMap.h>

// Map Building algorithms:
#include <mrpt/slam/CMetricMapBuilderICP.h>
#include <mrpt/slam/CMetricMapBuilderRBPF.h>
#include <mrpt/slam/CRangeBearingKFSLAM.h>
#include <mrpt/slam/CRangeBearingKFSLAM2D.h>

// Observations:
#include <mrpt/slam/CObservation.h>
#include <mrpt/slam/CObservation2DRangeScan.h>
#include <mrpt/slam/CObservation3DRangeScan.h>
#include <mrpt/slam/CObservationRange.h>
#include <mrpt/slam/CObservationImage.h>
#include <mrpt/slam/CObservationVisualLandmarks.h>
#include <mrpt/slam/CObservationStereoImages.h>
#include <mrpt/slam/CObservationBeaconRanges.h>
#include <mrpt/slam/CObservationGasSensors.h>
#include <mrpt/slam/CObservationWirelessPower.h>
#include <mrpt/slam/CObservationRFID.h>
#include <mrpt/slam/CObservationGPS.h>
#include <mrpt/slam/CObservationBatteryState.h>
#include <mrpt/slam/CObservationIMU.h>
#include <mrpt/slam/CObservationOdometry.h>
#include <mrpt/slam/CObservationBearingRange.h>
#include <mrpt/slam/CObservationComment.h>

#include <mrpt/slam/observations_overlap.h>

#include <mrpt/slam/CSensoryFrame.h>

// Actions:
#include <mrpt/slam/CActionCollection.h>
#include <mrpt/slam/CActionRobotMovement2D.h>
#include <mrpt/slam/CActionRobotMovement3D.h>


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
