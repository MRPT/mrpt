/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
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
//				Benchmark: Graphs
// ------------------------------------------------------

template <class EDGE_TYPE,class MAPIMPL>
double graphs_test_populate(int nEdges, int _N)
{
	const long N = _N;

	std::vector< mrpt::graphs::CNetworkOfPoses<EDGE_TYPE,MAPIMPL> > gs(N);

	CTicTac	 tictac;
	for (long i=0;i<N;i++)
	{
		mrpt::graphs::CNetworkOfPoses<EDGE_TYPE,MAPIMPL>  &g = gs[i];
		for (int j=0;j<nEdges;++j)
		{
			g.insertEdge(j,j+1, EDGE_TYPE() );
		}
	}
	return tictac.Tac()/N;
}

template <class EDGE_TYPE,class MAPIMPL>
double graphs_test_populate_at_end(int nEdges, int _N)
{
	const long N = _N;

	std::vector< mrpt::graphs::CNetworkOfPoses<EDGE_TYPE,MAPIMPL> > gs(N);

	CTicTac	 tictac;
	for (long i=0;i<N;i++)
	{
		mrpt::graphs::CNetworkOfPoses<EDGE_TYPE,MAPIMPL>  &g = gs[i];
		for (int j=0;j<nEdges;++j)
		{
			//g.insertEdgeAtEnd(j,j+1, EDGE_TYPE() );
			g.edges.insert(g.edges.end(),std::make_pair(std::make_pair(j,j+1), EDGE_TYPE() ));
		}
	}
	return tictac.Tac()/N;
}

template <class EDGE_TYPE, class MAPS_IMPLEMENTATION>
double graphs_dijkstra(int nNodes, int _N)
{
	const long N = _N;

	randomGenerator.randomize(111);
	// Generate random graph:
	typedef mrpt::graphs::CNetworkOfPoses<EDGE_TYPE,MAPS_IMPLEMENTATION> graph_t;
	graph_t gs;
	{
		double edge_node_ratio = 2.0;
		for (unsigned int i=0;i<(unsigned int)nNodes;i++)
		{
			size_t nEdges = 1 + ( mrpt::random::randomGenerator.drawUniform32bit() % int(edge_node_ratio-1) );
			for (TNodeID k=0;k<nEdges;k++)
			{
				TNodeID dest;
				dest=i+1;
				if (k>0)
				{
					while (dest==i && dest!=i+1)	// Avoid self-loops!
						dest = mrpt::random::randomGenerator.drawUniform32bit()  % nNodes;
				}
				gs.insertEdge(i, dest, EDGE_TYPE() );
			}
		}
	}

	CTimeLogger tims;
	for (long i=0;i<N;i++)
	{
		tims.enter("op");
		mrpt::graphs::CDijkstra<graph_t> dij(gs, TNodeID(0) );
		tims.leave("op");
		// Don't count the time of the destructor.
	}
	tims.enable(false);
	double ret = tims.getMeanTime("op");
	tims.clear(true /* deep clear */);
	return ret;
}


// ------------------------------------------------------
// register_tests_graph
// ------------------------------------------------------
void register_tests_graph()
{
	randomGenerator.randomize(1234);

	lstTests.push_back( TestData("graph(2d): insertEdge x 1e3",graphs_test_populate<CPose2D,map_traits_stdmap>, 1e3,   1000) );
	lstTests.push_back( TestData("graph(2d,vec): insertEdge x 1e3",graphs_test_populate<CPose2D,map_traits_map_as_vector>, 1e3,   1000) );

	lstTests.push_back( TestData("graph(2d): insertEdgeAtEnd x 1e3",graphs_test_populate_at_end<CPose2D,map_traits_stdmap>, 1e3,   1000) );
	lstTests.push_back( TestData("graph(2d,vec): insertEdgeAtEnd x 1e3",graphs_test_populate_at_end<CPose2D,map_traits_map_as_vector>, 1e3,   1000) );

	lstTests.push_back( TestData("graph(2d pdf): insertEdge x 1e3",graphs_test_populate<CPosePDFGaussianInf,map_traits_stdmap>, 1e3,   1000) );
	lstTests.push_back( TestData("graph(2d pdf,vec): insertEdge x 1e3",graphs_test_populate<CPosePDFGaussianInf,map_traits_map_as_vector>, 1e3,   1000) );

	lstTests.push_back( TestData("graph(2d pdf): insertEdgeAtEnd x 1e3",graphs_test_populate_at_end<CPosePDFGaussianInf,map_traits_stdmap>, 1e3,   1000) );
	lstTests.push_back( TestData("graph(2d pdf,vec): insertEdgeAtEnd x 1e3",graphs_test_populate_at_end<CPosePDFGaussianInf,map_traits_map_as_vector>, 1e3,   1000) );

	lstTests.push_back( TestData("graph(2d): insertEdge x 1e4",graphs_test_populate<CPose2D,map_traits_stdmap>, 1e4,   250) );
	lstTests.push_back( TestData("graph(2d,vec): insertEdge x 1e4",graphs_test_populate<CPose2D,map_traits_map_as_vector>, 1e4,   250) );

	lstTests.push_back( TestData("graph(2d): insertEdgeAtEnd x 1e4",graphs_test_populate_at_end<CPose2D,map_traits_stdmap>, 1e4,   250) );
	lstTests.push_back( TestData("graph(2d,vec): insertEdgeAtEnd x 1e4",graphs_test_populate_at_end<CPose2D,map_traits_map_as_vector>, 1e4,   250) );

	lstTests.push_back( TestData("graph(2d pdf): insertEdge x 1e4",graphs_test_populate<CPosePDFGaussianInf,map_traits_stdmap>, 1e4,   250) );
	lstTests.push_back( TestData("graph(2d pdf,vec): insertEdge x 1e4",graphs_test_populate<CPosePDFGaussianInf,map_traits_map_as_vector>, 1e4,   250) );

	lstTests.push_back( TestData("graph(2d pdf): insertEdgeAtEnd x 1e4",graphs_test_populate_at_end<CPosePDFGaussianInf,map_traits_stdmap>, 1e4,   250) );
	lstTests.push_back( TestData("graph(2d pdf,vec): insertEdgeAtEnd x 1e4",graphs_test_populate_at_end<CPosePDFGaussianInf,map_traits_map_as_vector>, 1e4,   250) );

	lstTests.push_back( TestData("graph(3d): insertEdge x 1e3",graphs_test_populate<CPose3D,map_traits_stdmap>, 1e3,   1000) );
	lstTests.push_back( TestData("graph(3d,vec): insertEdge x 1e3",graphs_test_populate<CPose3D,map_traits_map_as_vector>, 1e3,   1000) );

	lstTests.push_back( TestData("graph(3d): insertEdgeAtEnd x 1e3",graphs_test_populate_at_end<CPose3D,map_traits_stdmap>, 1e3,   1000) );
	lstTests.push_back( TestData("graph(3d,vec): insertEdgeAtEnd x 1e3",graphs_test_populate_at_end<CPose3D,map_traits_map_as_vector>, 1e3,   1000) );

	lstTests.push_back( TestData("graph(3d pdf): insertEdge x 1e3",graphs_test_populate<CPose3DPDFGaussianInf,map_traits_stdmap>, 1e3,   1000) );
	lstTests.push_back( TestData("graph(3d pdf,vec): insertEdge x 1e3",graphs_test_populate<CPose3DPDFGaussianInf,map_traits_map_as_vector>, 1e3,   1000) );

	lstTests.push_back( TestData("graph(3d pdf): insertEdgeAtEnd x 1e3",graphs_test_populate_at_end<CPose3DPDFGaussianInf,map_traits_stdmap>, 1e3,   1000) );
	lstTests.push_back( TestData("graph(3d pdf,vec): insertEdgeAtEnd x 1e3",graphs_test_populate_at_end<CPose3DPDFGaussianInf,map_traits_map_as_vector>, 1e3,   1000) );

	lstTests.push_back( TestData("graph(3d): insertEdge x 1e4",graphs_test_populate<CPose3D,map_traits_stdmap>, 1e4,   250) );
	lstTests.push_back( TestData("graph(3d,vec): insertEdge x 1e4",graphs_test_populate<CPose3D,map_traits_map_as_vector>, 1e4,   250) );

	lstTests.push_back( TestData("graph(3d): insertEdgeAtEnd x 1e4",graphs_test_populate_at_end<CPose3D,map_traits_stdmap>, 1e4,   250) );
	lstTests.push_back( TestData("graph(3d,vec): insertEdgeAtEnd x 1e4",graphs_test_populate_at_end<CPose3D,map_traits_map_as_vector>, 1e4,   250) );

	lstTests.push_back( TestData("graph(3d pdf): insertEdge x 1e4",graphs_test_populate<CPose3DPDFGaussianInf,map_traits_stdmap>, 1e4,   250) );
	lstTests.push_back( TestData("graph(3d pdf,vec): insertEdge x 1e4",graphs_test_populate<CPose3DPDFGaussianInf,map_traits_map_as_vector>, 1e4,   250) );

	lstTests.push_back( TestData("graph(3d pdf): insertEdgeAtEnd x 1e4",graphs_test_populate_at_end<CPose3DPDFGaussianInf,map_traits_stdmap>, 1e4,   250) );
	lstTests.push_back( TestData("graph(3d pdf,vec): insertEdgeAtEnd x 1e4",graphs_test_populate_at_end<CPose3DPDFGaussianInf,map_traits_map_as_vector>, 1e4,   250) );


	lstTests.push_back( TestData("graph(3d): dijkstra 1e2 nodes",graphs_dijkstra<CPose3D,map_traits_stdmap>, 1e2, 500) );
	lstTests.push_back( TestData("graph(3d,vec): dijkstra 1e2 nodes",graphs_dijkstra<CPose3D,map_traits_map_as_vector>, 1e2, 500) );

	lstTests.push_back( TestData("graph(3d): dijkstra 1e3 nodes",graphs_dijkstra<CPose3D,map_traits_stdmap>, 1e3, 500) );
	lstTests.push_back( TestData("graph(3d,vec): dijkstra 1e3 nodes",graphs_dijkstra<CPose3D,map_traits_map_as_vector>, 1e3, 500) );

	lstTests.push_back( TestData("graph(3d): dijkstra 1e4 nodes",graphs_dijkstra<CPose3D,map_traits_stdmap>, 1e4, 50) );
	lstTests.push_back( TestData("graph(3d,vec): dijkstra 1e4 nodes",graphs_dijkstra<CPose3D,map_traits_map_as_vector>, 1e4, 50) );

	lstTests.push_back( TestData("graph(3d): dijkstra 1e5 nodes",graphs_dijkstra<CPose3D,map_traits_stdmap>, 1e5, 50) );
	lstTests.push_back( TestData("graph(3d,vec): dijkstra 1e5 nodes",graphs_dijkstra<CPose3D,map_traits_map_as_vector>, 1e5, 50) );

	lstTests.push_back( TestData("graph(2d): dijkstra 1e5 nodes",graphs_dijkstra<CPose2D,map_traits_stdmap>, 1e5, 50) );
	lstTests.push_back( TestData("graph(2d,vec): dijkstra 1e5 nodes",graphs_dijkstra<CPose2D,map_traits_map_as_vector>, 1e5, 50) );
}
