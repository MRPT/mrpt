/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#ifndef MRPT_NO_WARN_BIG_HDR
MRPT_WARNING(
	"Including <mrpt/slam.h> makes compilation much slower, consider including "
	"only what you need (define MRPT_NO_WARN_BIG_HDR to disable this warning)")
#endif

#include <mrpt/config.h>

// Maps:
#include <mrpt/maps.h>
#include <mrpt/maps/CMultiMetricMap.h>	// This class is in [mrpt-slam]
#include <mrpt/maps/CMultiMetricMapPDF.h>  // This class is in [mrpt-slam]

// Map Building algorithms:
#include <mrpt/slam/CMetricMapBuilderICP.h>
#include <mrpt/slam/CMetricMapBuilderRBPF.h>
#include <mrpt/slam/CRangeBearingKFSLAM.h>
#include <mrpt/slam/CRangeBearingKFSLAM2D.h>

// Observations:
#include <mrpt/obs.h>
#include <mrpt/obs/CObservationVisualLandmarks.h>  // In [mrpt-vision]

// Algorithms:
#include <mrpt/slam/CGridMapAligner.h>
#include <mrpt/slam/CICP.h>
#include <mrpt/slam/CIncrementalMapPartitioner.h>
#include <mrpt/slam/CMonteCarloLocalization2D.h>
#include <mrpt/slam/CMonteCarloLocalization3D.h>
#include <mrpt/slam/CRejectionSamplingRangeOnlyLocalization.h>
#include <mrpt/slam/data_association.h>

// Others:
#include <mrpt/slam/COccupancyGridMapFeatureExtractor.h>
#include <mrpt/slam/observations_overlap.h>
