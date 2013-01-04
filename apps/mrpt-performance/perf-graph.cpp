/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
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

#include <mrpt/graphs.h>
#include <mrpt/graphslam.h>
#include <mrpt/random.h>
#include <mrpt/utils/CTicTac.h>
#include <mrpt/utils/CTimeLogger.h>

#include "common.h"

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace mrpt::graphs;
using namespace mrpt::graphslam;
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
	tims.clear();
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
