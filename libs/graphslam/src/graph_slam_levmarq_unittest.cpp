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
#include <mrpt/system/filesystem.h>

// Defined in tests/test_main.cpp
namespace mrpt
{
extern std::string MRPT_GLOBAL_UNITTEST_SRC_DIR;
}

using namespace mrpt;
using namespace mrpt::random;
using namespace mrpt::poses;
using namespace mrpt::graphs;
using namespace mrpt::math;
using namespace std;

// Define in/out files for testing:
using in_out_filenames = std::set<std::tuple<std::string, std::string>>;
const std::map<std::string, in_out_filenames> inout_graph_files{
	{"GraphTester2D",
	 {{"graphslam_SE2_in.graph", "graphslam_SE2_out_good.graph"},
	  {"graphslam_SE2_in2.graph", "graphslam_SE2_out_good2.graph"},
	  {"graphslam_SE2_in3.graph", "graphslam_SE2_out_good3.graph"}}},
	{"GraphTester2DInf",
	 {{"graphslam_SE2_in.graph", "graphslam_SE2_out_good.graph"},
	  {"graphslam_SE2pdf_in.graph", "graphslam_SE2pdf_out_good.graph"}}}};

template <class my_graph_t>
class GraphTester : public GraphSlamLevMarqTest<my_graph_t>,
					public ::testing::Test
{
   protected:
	void SetUp() override {}
	void TearDown() override {}
	void test_ring_path(const char* className)
	{
		// This is the initial input graph (make a copy for later use):
		my_graph_t graph;
		GraphSlamLevMarqTest<my_graph_t>::create_ring_path(graph);

		const my_graph_t graph_initial = graph;

		// ----------------------------
		//  Run graph slam:
		// ----------------------------
		mrpt::system::TParametersDouble params;
		// params["verbose"] = 1;
		params["max_iterations"] = 100;

		graphslam::TResultInfoSpaLevMarq levmarq_info;

		graphslam::optimize_graph_spa_levmarq(
			graph, levmarq_info, nullptr, params);

		const double err_init = graph_initial.chi2();
		const double err_end = graph.chi2();
		std::cout << "err_init: " << err_init << std::endl;
		std::cout << "err_end: " << err_end << std::endl;
		//		graph_initial.saveToTextFile(
		//			string("in_") + string(className) + string(".graph"));

		// Do some basic checks on the results:
		EXPECT_GE(levmarq_info.num_iters, 2U);
		EXPECT_LE(levmarq_info.final_total_sq_error, 5e-2);
		EXPECT_LT(err_end, err_init);

	}  // end test_ring_path

	void compare_two_graphs(
		const my_graph_t& g1, const my_graph_t& g2,
		const double eps_node_pos = 1e-3, const double eps_edges = 1e-3)
	{
		EXPECT_EQ(g1.edges.size(), g2.edges.size());
		EXPECT_EQ(g1.nodes.size(), g2.nodes.size());
		EXPECT_EQ(g1.root, g2.root);

		if (g1.edges.size() != g2.edges.size() ||
			g1.nodes.size() != g2.nodes.size())
			return;

		// Check that the edge values are OK:
		{
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
						.maxCoeff(),
					eps_edges);
			}
		}

		// Check nodes:
		{
			auto itn1 = g1.nodes.cbegin(), itn2 = g2.nodes.cbegin();
			for (; itn1 != g1.nodes.cend(); ++itn1, ++itn2)
			{
				EXPECT_EQ(itn1->first, itn2->first);
				EXPECT_NEAR(
					0,
					(itn1->second.getAsVectorVal() -
					 itn2->second.getAsVectorVal())
						.array()
						.abs()
						.maxCoeff(),
					eps_node_pos)
					<< "Poses of keyframe #" << itn1->first
					<< " do not match:" << std::endl
					<< "- Expected: " << itn2->second << std::endl
					<< "- Got     : " << itn1->second << std::endl;
			}
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

	void test_optimize_compare_known_solution(const char* type)
	{
		auto files_it = inout_graph_files.find(type);
		if (files_it == inout_graph_files.end())
			return;  // No tests for this type

		const string prefix = MRPT_GLOBAL_UNITTEST_SRC_DIR + string("/tests/");
		for (const auto& tst : files_it->second)
		{
			std::cout << "Testing graph type `" << type << "`, in_file=`"
					  << std::get<0>(tst) << "`" << std::endl;

			const string in_f = prefix + std::get<0>(tst);
			ASSERT_FILE_EXISTS_(in_f);
			const string good_f = prefix + std::get<1>(tst);
			ASSERT_FILE_EXISTS_(good_f);

			my_graph_t graph, graph_good;
			graph.loadFromTextFile(in_f);
			graph_good.loadFromTextFile(good_f);
			ASSERT_(graph.nodeCount() > 1);
			ASSERT_EQ(graph.nodeCount(), graph_good.nodeCount());
			ASSERT_EQ(graph.edgeCount(), graph_good.edgeCount());

			// Optimize:
			const my_graph_t graph_initial = graph;
			mrpt::system::TParametersDouble params;
			params["max_iterations"] = 100;

			graphslam::TResultInfoSpaLevMarq levmarq_info;

			graphslam::optimize_graph_spa_levmarq(
				graph, levmarq_info, nullptr, params);

			/* DEBUG */
			const double err_init = graph_initial.chi2();
			const double err_end = graph.chi2();
			const double err_good = graph_good.chi2();
			std::cout << "err_init: " << err_init << std::endl;
			std::cout << "err_end: " << err_end << std::endl;
			std::cout << "err_good: " << err_good << std::endl;

			// Do some basic checks on the results:
			EXPECT_GE(levmarq_info.num_iters, 2U);
			EXPECT_LE(levmarq_info.final_total_sq_error, 0.2);
			EXPECT_LT(err_end, err_init);

			// Compare to good solution:
			compare_two_graphs(graph, graph_good);
		}
	}
};

using GraphTester2D = GraphTester<CNetworkOfPoses2D>;
using GraphTester3D = GraphTester<CNetworkOfPoses3D>;
using GraphTester2DInf = GraphTester<CNetworkOfPoses2DInf>;
using GraphTester3DInf = GraphTester<CNetworkOfPoses3DInf>;

#define GRAPHS_TESTS(_TYPE)                           \
	TEST_F(_TYPE, OptimizeSampleRingPath)             \
	{                                                 \
		for (int seed = 1; seed <= 3; seed++)         \
		{                                             \
			getRandomGenerator().randomize(seed);     \
			test_ring_path(#_TYPE);                   \
		}                                             \
	}                                                 \
	TEST_F(_TYPE, BinarySerialization)                \
	{                                                 \
		getRandomGenerator().randomize(123);          \
		test_graph_bin_serialization();               \
	}                                                 \
	TEST_F(_TYPE, WriteReadTextFile)                  \
	{                                                 \
		getRandomGenerator().randomize(123);          \
		test_graph_text_serialization();              \
	}                                                 \
	TEST_F(_TYPE, OptimizeCompareKnownSolution)       \
	{                                                 \
		test_optimize_compare_known_solution(#_TYPE); \
	}

MRPT_TODO("Re-enable tests after https://github.com/MRPT/mrpt/issues/770");

GRAPHS_TESTS(GraphTester2D)
// GRAPHS_TESTS(GraphTester3D)
GRAPHS_TESTS(GraphTester2DInf)
// GRAPHS_TESTS(GraphTester3DInf)
