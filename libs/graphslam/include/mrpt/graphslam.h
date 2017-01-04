/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef MRPT_NO_WARN_BIG_HDR
#include <mrpt/utils/core_defs.h>
MRPT_WARNING("Including <mrpt/graphslam.h> makes compilation much slower, consider including only what you need (define MRPT_NO_WARN_BIG_HDR to disable this warning)")
#endif

#ifndef _mrpt_graphslam_H
#define _mrpt_graphslam_H

// Graph SLAM: Common headers
#include "graphslam/types.h"

// Graph SLAM: Batch solvers
#include "graphslam/levmarq.h"

// Interfaces for implementing deciders/optimizers
#include "graphslam/interfaces/CRegistrationDeciderOrOptimizer.h"
#include "graphslam/interfaces/CNodeRegistrationDecider.h"
#include "graphslam/interfaces/CEdgeRegistrationDecider.h"
#include "graphslam/interfaces/CGraphSlamOptimizer.h"

// Node Registration Deciders
#include "graphslam/NRD/CEmptyNRD.h"
#include "graphslam/NRD/CFixedIntervalsNRD.h"
#include "graphslam/NRD/CICPCriteriaNRD.h"

// Edge Registration Deciders
#include "graphslam/ERD/CEmptyERD.h"
#include "graphslam/ERD/CICPCriteriaERD.h"
#include "graphslam/ERD/CLoopCloserERD.h"

// GraphSlamOptimizers
#include "graphslam/GSO/CLevMarqGSO.h"

// Graph SLAM Engine - Relevant headers
#include "graphslam/misc/CRangeScanRegistrationDecider.h"
#include "graphslam/misc/CEdgeCounter.h"
#include "graphslam/misc/CWindowManager.h"
#include "graphslam/misc/CWindowObserver.h"
#include "graphslam/misc/TSlidingWindow.h"

// App-Related headers
#include "graphslam/apps_related/TUserOptionsChecker.h"
#include "graphslam/apps_related/CGraphSlamHandler.h"

// Main graphslam-engine header
#include "graphslam/CGraphSlamEngine.h"



#endif
