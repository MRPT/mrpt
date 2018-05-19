/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "graph_slam_levmarq_test_common.h"

#include <gtest/gtest.h>
#include <mrpt/serialization/CArchive.h>

using namespace mrpt;
using namespace mrpt::random;
using namespace mrpt::poses;
using namespace mrpt::graphs;
using namespace mrpt::math;
using namespace std;

template <class my_graph_t>
class GraphTester : public GraphSlamLevMarqTest<my_graph_t>,
					public ::testing::Test
{
   protected:
	virtual void SetUp() {}
	virtual void TearDown() {}
	void test_ring_path()
	{
		// This is the initial input graph (make a copy for later use):
		my_graph_t graph;
		GraphSlamLevMarqTest<my_graph_t>::create_ring_path(graph);

		const my_graph_t graph_initial = graph;

		// ----------------------------
		//  Run graph slam:
		// ----------------------------
		mrpt::system::TParametersDouble params;
		// params["verbose"]  = 1;
		// params["profiler"] = 1;
		params["max_iterations"] = 1000;

		graphslam::TResultInfoSpaLevMarq levmarq_info;

		graphslam::optimize_graph_spa_levmarq(
			graph, levmarq_info, nullptr, params);

		const double err_init = graph_initial.chi2();
		const double err_end = graph.chi2();
		graph_initial.saveToTextFile("in.graph");
		graph.saveToTextFile("out.graph");

		// Do some basic checks on the results:
		EXPECT_GE(levmarq_info.num_iters, 10U);
		EXPECT_LE(levmarq_info.final_total_sq_error, 1e-2);
		EXPECT_LT(err_end, err_init);

	}  // end test_ring_path

	void compare_two_graphs(const my_graph_t& g1, const my_graph_t& g2)
	{
		EXPECT_EQ(g1.edges.size(), g2.edges.size());
		EXPECT_EQ(g1.nodes.size(), g2.nodes.size());
		EXPECT_EQ(g1.root, g2.root);

		if (g1.edges.size() != g2.edges.size() ||
			g1.nodes.size() != g2.nodes.size())
			return;

		// Also check that the edge values are OK:
		typename my_graph_t::const_iterator it1, it2;
		for (it1 = g1.edges.begin(), it2 = g2.edges.begin();
			 it1 != g1.edges.end(); ++it1, ++it2)
		{
			EXPECT_EQ(it1->first, it2->first);
			EXPECT_NEAR(
				0,
				(it1->second.getPoseMean().getAsVectorVal() -
				 it2->second.getPoseMean().getAsVectorVal())
					.array()
					.abs()
					.sum(),
				1e-9);
		}
	}

	void test_graph_text_serialization()
	{
		my_graph_t graph;
		GraphSlamLevMarqTest<my_graph_t>::create_ring_path(graph);
		// text write:
		std::stringstream ss;
		graph.writeAsText(ss);
		// read:
		my_graph_t read_graph;
		ss.seekg(0);  // rewind
		read_graph.readAsText(ss);

		compare_two_graphs(graph, read_graph);
	}

	void test_graph_bin_serialization()
	{
		my_graph_t graph;
		GraphSlamLevMarqTest<my_graph_t>::create_ring_path(graph);
		// binary write:
		mrpt::io::CMemoryStream mem;
		auto arch = mrpt::serialization::archiveFrom(mem);
		arch << graph;
		// read:
		my_graph_t read_graph;
		mem.Seek(0);
		arch >> read_graph;

		compare_two_graphs(graph, read_graph);
	}
};

using GraphTester2D = GraphTester<CNetworkOfPoses2D>;
using GraphTester3D = GraphTester<CNetworkOfPoses3D>;
using GraphTester2DInf = GraphTester<CNetworkOfPoses2DInf>;
using GraphTester3DInf = GraphTester<CNetworkOfPoses3DInf>;

#define GRAPHS_TESTS(_TYPE)                       \
	TEST_F(_TYPE, OptimizeSampleRingPath)         \
	{                                             \
		for (int seed = 1; seed < 3; seed++)      \
		{                                         \
			getRandomGenerator().randomize(seed); \
			test_ring_path();                     \
		}                                         \
	}                                             \
	TEST_F(_TYPE, BinarySerialization)            \
	{                                             \
		getRandomGenerator().randomize(123);      \
		test_graph_bin_serialization();           \
	}                                             \
	TEST_F(_TYPE, WriteReadTextFile)              \
	{                                             \
		getRandomGenerator().randomize(123);      \
		test_graph_text_serialization();          \
	}

GRAPHS_TESTS(GraphTester2D)
GRAPHS_TESTS(GraphTester3D)
GRAPHS_TESTS(GraphTester2DInf)
GRAPHS_TESTS(GraphTester3DInf)
