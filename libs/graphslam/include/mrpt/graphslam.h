/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef _mrpt_graphslam_H
#define _mrpt_graphslam_H

// Graph SLAM: Common headers
#include "graphslam/types.h"

// Graph SLAM: Batch solvers
#include "graphslam/levmarq.h"

// Interfaces for implementing deciders/optimizers
#include "graphslam/CRegistrationDeciderOrOptimizer.h"
#include "graphslam/CRangeScanRegistrationDecider.h"
#include "graphslam/CNodeRegistrationDecider.h"
#include "graphslam/CEdgeRegistrationDecider.h"
#include "graphslam/CGraphSlamOptimizer.h"

// Node Registration Deciders
#include "graphslam/CEmptyNRD.h"
#include "graphslam/CFixedIntervalsNRD.h"
#include "graphslam/CICPCriteriaNRD.h"

// Edge Registration Deciders
#include "graphslam/CEmptyERD.h"
#include "graphslam/CICPCriteriaERD.h"
#include "graphslam/CLoopCloserERD.h"

// GraphSlamOptimizers
#include "graphslam/CLevMarqGSO.h"

// Graph SLAM Engine - Relevant headers
#include "graphslam/CEdgeCounter.h"
#include "graphslam/CWindowManager.h"
#include "graphslam/CWindowObserver.h"
#include "graphslam/TSlidingWindow.h"

#include "graphslam/CGraphSlamEngine.h"



#endif
