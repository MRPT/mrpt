/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */


#include <mrpt/graphslam.h>
#include <mrpt/graphs.h>
#include <mrpt/random.h>
#include <mrpt/utils/CMemoryStream.h>
#include <gtest/gtest.h>

using namespace mrpt;
using namespace mrpt::random;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::graphs;
using namespace mrpt::math;
using namespace std;


template <class my_graph_t>
class GraphSlamLevMarqTester : public ::testing::Test {
protected:
	virtual void SetUp()
	{
	}
	virtual void TearDown() {  }

	// adds a new edge to the graph. The edge is annotated with the relative position of the two nodes
	static void addEdge(TNodeID from, TNodeID to, const typename my_graph_t::global_poses_t &real_poses,my_graph_t &graph)
	{
		typename my_graph_t::edge_t RelativePose = real_poses.find(to)->second - real_poses.find(from)->second;
		graph.insertEdge(from,to, RelativePose );
	}

	// The graph: nodes + edges:
	void create_ring_path(my_graph_t & graph)
	{
		// The global poses of each node (without covariance):
		typename my_graph_t::global_poses_t  real_node_poses;

		// ----------------------------
		// Create a random graph:
		// ----------------------------
		const size_t N_VERTEX = 50;
		const double DIST_THRES = 7;
		const double NODES_XY_MAX = 20;

		// Level of noise in nodes initial positions:
		const double STD_NOISE_NODE_XYZ = 0.5;
		const double STD_NOISE_NODE_ANG = DEG2RAD(5);

		// Level of noise in edges:
		const double STD_NOISE_EDGE_XYZ = 0; //0.01;
		const double STD_NOISE_EDGE_ANG = 0; //DEG2RAD(0.1);


		for (TNodeID j=0;j<N_VERTEX;j++)
		{
			static double ang = 2*M_PI/N_VERTEX;
			const double R = NODES_XY_MAX + 2 * (j % 2 ? 1:-1);
			CPose2D p(
				R*cos(ang*j),
				R*sin(ang*j),
				ang);

			// Save real pose:
			real_node_poses[j] = p;

			// Copy the nodes to the graph, and add some noise:
			graph.nodes[j] = p;
		}


		// Add some edges
		for (TNodeID i=0;i<N_VERTEX;i++)
		{
			for (TNodeID j=i+1;j<N_VERTEX;j++)
			{
				if ( real_node_poses[i].distanceTo(real_node_poses[j]) < DIST_THRES )
					addEdge(i,j,real_node_poses,graph);
			}
		}

		// Cross-links:
		addEdge(0,N_VERTEX/2,real_node_poses,graph);

		// The root node (the origin of coordinates):
		graph.root = TNodeID(0);

		// This is the ground truth graph (make a copy for later use):
		const my_graph_t  graph_GT = graph;

		// Add noise to edges & nodes:
		for (typename my_graph_t::edges_map_t::iterator itEdge=graph.edges.begin();itEdge!=graph.edges.end();++itEdge)
			itEdge->second += typename my_graph_t::edge_t( CPose3D(
				randomGenerator.drawGaussian1D(0,STD_NOISE_EDGE_XYZ),
				randomGenerator.drawGaussian1D(0,STD_NOISE_EDGE_XYZ),
				randomGenerator.drawGaussian1D(0,STD_NOISE_EDGE_XYZ),
				randomGenerator.drawGaussian1D(0,STD_NOISE_EDGE_ANG),
				randomGenerator.drawGaussian1D(0,STD_NOISE_EDGE_ANG),
				randomGenerator.drawGaussian1D(0,STD_NOISE_EDGE_ANG) ) );

		for (typename my_graph_t::global_poses_t::iterator itNode=graph.nodes.begin();itNode!=graph.nodes.end();++itNode)
			if (itNode->first!=graph.root)
				itNode->second += typename my_graph_t::edge_t::type_value( CPose3D(
					randomGenerator.drawGaussian1D(0,STD_NOISE_NODE_XYZ),
					randomGenerator.drawGaussian1D(0,STD_NOISE_NODE_XYZ),
					randomGenerator.drawGaussian1D(0,STD_NOISE_NODE_XYZ),
					randomGenerator.drawGaussian1D(0,STD_NOISE_NODE_ANG),
					randomGenerator.drawGaussian1D(0,STD_NOISE_NODE_ANG),
					randomGenerator.drawGaussian1D(0,STD_NOISE_NODE_ANG) ) );
	}

	void test_ring_path()
	{
		// This is the initial input graph (make a copy for later use):
		my_graph_t graph;
		create_ring_path(graph);

		const my_graph_t  graph_initial = graph;

		// ----------------------------
		//  Run graph slam:
		// ----------------------------
		TParametersDouble  params;
		//params["verbose"]  = 1;
		//params["profiler"] = 1;
		params["max_iterations"] = 1000;

		graphslam::TResultInfoSpaLevMarq  levmarq_info;

		graphslam::optimize_graph_spa_levmarq(
			graph,
			levmarq_info,
			NULL,
			params
			);

		// Do some basic checks on the results:
		EXPECT_GE(levmarq_info.num_iters, 10U);
		EXPECT_LE(levmarq_info.final_total_sq_error, 1e-2);

	} // end test_ring_path

	void test_graph_bin_serialization()
	{
		my_graph_t graph;
		create_ring_path(graph);

		// binary dump:
		mrpt::utils::CMemoryStream mem;
		mem << graph;

		{
			my_graph_t read_graph;
			mem.Seek(0);
			mem >> read_graph;

			EXPECT_EQ( read_graph.edges.size(), graph.edges.size() );
			EXPECT_EQ( read_graph.nodes.size(), graph.nodes.size() );

			// Also check that the edge values are OK:
			typename my_graph_t::const_iterator it1, it2;
			for (it1=read_graph.edges.begin(), it2=graph.edges.begin(); it1!=read_graph.edges.end(); ++it1, ++it2)
			{
				EXPECT_EQ(it1->first, it2->first);
				EXPECT_NEAR(0, (it1->second.getPoseMean().getAsVectorVal() - it2->second.getPoseMean().getAsVectorVal() ).array().abs().sum(), 1e-9 );
			}
		}

	}

};

typedef GraphSlamLevMarqTester<CNetworkOfPoses2D> GraphSlamLevMarqTester2D;
typedef GraphSlamLevMarqTester<CNetworkOfPoses3D> GraphSlamLevMarqTester3D;

TEST_F(GraphSlamLevMarqTester2D, OptimizeSampleRingPath)
{
	for (int seed=1;seed<5;seed++)
	{
		randomGenerator.randomize(seed);
		test_ring_path();
	}
}
TEST_F(GraphSlamLevMarqTester2D, BinarySerialization)
{
	randomGenerator.randomize(123);
	test_graph_bin_serialization();
}

TEST_F(GraphSlamLevMarqTester3D, OptimizeSampleRingPath)
{
	for (int seed=1;seed<5;seed++)
	{
		randomGenerator.randomize(seed);
		test_ring_path();
	}
}
TEST_F(GraphSlamLevMarqTester3D, BinarySerialization)
{
	randomGenerator.randomize(123);
	test_graph_bin_serialization();
}
