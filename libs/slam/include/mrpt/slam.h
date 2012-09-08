/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | Copyright (c) 2012, University of Almeria                                 |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
   +---------------------------------------------------------------------------+ */
#ifndef mrpt_slam_H
#define mrpt_slam_H

#include <mrpt/config.h>

// Only really include all headers if we come from a user program (anything
//  not defining mrpt_*_EXPORTS) or MRPT is being built with precompiled headers.
#if !defined(mrpt_slam_EXPORTS) || MRPT_ENABLE_PRECOMPILED_HDRS || defined(MRPT_ALWAYS_INCLUDE_ALL_HEADERS)

// This is to try to avoid an internal compiler error in MSVC 2008... :-(
#if defined(mrpt_slam_EXPORTS) && MRPT_ENABLE_PRECOMPILED_HDRS
	#include <mrpt/base.h>
#endif

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

#include <mrpt/slam/CDetectorDoorCrossing.h>
#include <mrpt/slam/CIncrementalMapPartitioner.h>
#include <mrpt/slam/CPathPlanningMethod.h>
#include <mrpt/slam/CPathPlanningCircularRobot.h>
#include <mrpt/slam/CRejectionSamplingRangeOnlyLocalization.h>

#include <mrpt/slam/data_association.h>

// PDFs:
#include <mrpt/slam/CMultiMetricMapPDF.h>

// Others:
#include <mrpt/slam/CRawlog.h>

#include <mrpt/slam/COccupancyGridMapFeatureExtractor.h>

#endif // end precomp.headers

#endif
