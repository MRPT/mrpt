/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/graphs/CNetworkOfPoses.h>
#include <mrpt/graphs/dijkstra.h>
#include <mrpt/random.h>
#include <mrpt/utils/CTimeLogger.h>

#include "common.h"

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace mrpt::graphs;
using namespace mrpt::poses;
using namespace mrpt::random;
using namespace std;


// ------------------------------------------------------
//				Benchmark: srba
// ------------------------------------------------------


// ------------------------------------------------------
// register_tests_srba
// ------------------------------------------------------
void register_tests_srba()
{
	randomGenerator.randomize(1234);

//	lstTests.push_back( TestData("graph(2d): insertEdge x 1e3",graphs_test_populate<CPose2D,map_traits_stdmap>, 1e3,   1000) );

}
