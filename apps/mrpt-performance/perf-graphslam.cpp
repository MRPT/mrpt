/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

// Reuse code from unit test:
#include "../../libs/graphslam/src/graph_slam_levmarq_test_common.h"

#include "common.h"

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace mrpt::graphs;
using namespace mrpt::poses;
using namespace mrpt::random;
using namespace std;


// ------------------------------------------------------
//				Benchmark: Graph-SLAM
// ------------------------------------------------------

template <class GRAPH_TYPE>
double graphslam_levmarq_solve(int nVertices, int N)
{
	// This is the initial input graph (make a copy for later use):
	GRAPH_TYPE graph;
	GraphSlamLevMarqTest<GRAPH_TYPE>::create_ring_path(graph, nVertices);

	//cout << graph.nodeCount() << " nodes, " << graph.edgeCount() << " edges\n";


	TParametersDouble  params;
	//params["verbose"]  = 1;
	//params["profiler"] = 1;
	params["max_iterations"] = 1000;

	CTimeLogger timer;

	for (long i=0;i<N;i++)
	{
		GRAPH_TYPE  graph0 = graph;

		graphslam::TResultInfoSpaLevMarq  levmarq_info;

		timer.enter("test");

		graphslam::optimize_graph_spa_levmarq(
			graph0,
			levmarq_info,
			NULL,
			params
			);
		timer.leave("test");
	}
	const double ret =timer.getMeanTime("test");
	timer.clear(true); // this disables dump to cout upon destruction
	return ret;
}


// ------------------------------------------------------
// register_tests_graphslam
// ------------------------------------------------------
void register_tests_graphslam()
{
	randomGenerator.randomize(1234);
	lstTests.push_back( TestData("graphslam(2d): levmarq 50 KFs/101 edges",graphslam_levmarq_solve<CNetworkOfPoses2D>, 50, 50) );
	lstTests.push_back( TestData("graphslam(2d): levmarq 100 KFs/451 edges",graphslam_levmarq_solve<CNetworkOfPoses2D>, 100, 2) );
	lstTests.push_back( TestData("graphslam(3d): levmarq 50 KFs/101 edges",graphslam_levmarq_solve<CNetworkOfPoses3D>, 50, 10) );
	lstTests.push_back( TestData("graphslam(3d): levmarq 100 KFs/451 edges",graphslam_levmarq_solve<CNetworkOfPoses3D>, 100, 2) );

}
