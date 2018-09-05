/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#pragma once

// Graph SLAM: Common headers
#include "graphslam/types.h"

// Graph SLAM: Batch solvers
#include "graphslam/levmarq.h"

// Interfaces for implementing deciders/optimizers
#include "graphslam/interfaces/CRegistrationDeciderOrOptimizer.h"
#include "graphslam/interfaces/CNodeRegistrationDecider.h"
#include "graphslam/interfaces/CEdgeRegistrationDecider.h"
#include "graphslam/interfaces/CRangeScanEdgeRegistrationDecider.h"
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
#include "graphslam/GSO/CEmptyGSO.h"
#include "graphslam/GSO/CLevMarqGSO.h"

// Graph SLAM Engine - Relevant headers
#include "graphslam/misc/CRangeScanOps.h"
#include "graphslam/misc/CEdgeCounter.h"
#include "graphslam/misc/CWindowManager.h"
#include "graphslam/misc/CWindowObserver.h"
#include "graphslam/misc/TSlidingWindow.h"
#include "graphslam/misc/TUncertaintyPath.h"

// App-Related headers
#include "graphslam/apps_related/TUserOptionsChecker.h"
#include "graphslam/apps_related/CGraphSlamHandler.h"

// Main graphslam-engine header
#include "graphslam/CGraphSlamEngine.h"
