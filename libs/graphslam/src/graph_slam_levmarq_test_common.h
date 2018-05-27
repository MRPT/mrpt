/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include <mrpt/graphslam/types.h>
#include <mrpt/graphslam/levmarq.h>
#include <mrpt/graphs.h>
#include <mrpt/random.h>

using namespace mrpt;
using namespace mrpt::random;
using namespace mrpt::poses;
using namespace mrpt::graphs;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace std;

template <class my_graph_t>
class GraphSlamLevMarqTest
{
public:

	// SFINAE seems to be badly supported in MSVC 2015...sigh. Just do non-templatized overloading:
	template <class edge_t> struct EdgeAdder;

	template <> struct EdgeAdder<mrpt::poses::CPose2D>
	{
		template <class my_graph_t>
		static void add(
			TNodeID from, TNodeID to,
			const typename my_graph_t::global_poses_t& real_poses,
			my_graph_t& graph)
		{
			auto RelativePose =
				real_poses.find(to)->second - real_poses.find(from)->second;
			graph.insertEdge(from, to, RelativePose);
		}

		static void addNoise(mrpt::poses::CPose2D& p, const mrpt::poses::CPose3D & noise)
		{
			p += mrpt::poses::CPose2D(noise);
		}
	};
	template <> struct EdgeAdder<mrpt::poses::CPose3D>
	{
		template <class my_graph_t>
		static void add(
			TNodeID from, TNodeID to,
			const typename my_graph_t::global_poses_t& real_poses,
			my_graph_t& graph)
		{
			auto RelativePose =
				real_poses.find(to)->second - real_poses.find(from)->second;
			graph.insertEdge(from, to, RelativePose);
		}
		static void addNoise(mrpt::poses::CPose3D& p, const mrpt::poses::CPose3D & noise)
		{
			p += noise;
		}
	};
	template <> struct EdgeAdder<mrpt::poses::CPosePDFGaussianInf>
	{
		template <class my_graph_t>
		static void add(
			TNodeID from, TNodeID to,
			const typename my_graph_t::global_poses_t& real_poses,
			my_graph_t& graph)
		{
			auto RelativePose =
				real_poses.find(to)->second - real_poses.find(from)->second;
			// Add information matrix :
			const auto N = my_graph_t::edge_t::state_length;
			typedef mrpt::math::CMatrixFixedNumeric<double, N, N> InfMat;
			auto mat = randomGenerator
				.drawDefinitePositiveMatrix(
					N, 2.0 /*std*/, 1.0 /*diagonal offset*/);
			graph.insertEdge(
				from, to, typename my_graph_t::edge_t(RelativePose, mat));
		}
		static void addNoise(mrpt::poses::CPosePDFGaussianInf& p, const mrpt::poses::CPose3D & noise)
		{
			p.mean += mrpt::poses::CPose2D(noise);
		}
	};
	template <> struct EdgeAdder<mrpt::poses::CPose3DPDFGaussianInf>
	{
		template <class my_graph_t>
		static void add(
			TNodeID from, TNodeID to,
			const typename my_graph_t::global_poses_t& real_poses,
			my_graph_t& graph)
		{
			auto RelativePose =
				real_poses.find(to)->second - real_poses.find(from)->second;
			// Add information matrix :
			const auto N = my_graph_t::edge_t::state_length;
			typedef mrpt::math::CMatrixFixedNumeric<double, N, N> InfMat;
			auto mat = randomGenerator
				.drawDefinitePositiveMatrix(
					N, 2.0 /*std*/, 1.0 /*diagonal offset*/);
			graph.insertEdge(
				from, to, typename my_graph_t::edge_t(RelativePose, mat));
		}
		static void addNoise(mrpt::poses::CPose3DPDFGaussianInf& p, const mrpt::poses::CPose3D & noise)
		{
			p.mean += noise;
		}
	};

	// The graph: nodes + edges:
	static void create_ring_path(
		my_graph_t& graph, size_t N_VERTEX = 50, double DIST_THRES = 7,
		double NODES_XY_MAX = 20)
	{
		// The global poses of each node (without covariance):
		typename my_graph_t::global_poses_t real_node_poses;

		// ----------------------------
		// Create a random graph:
		// ----------------------------
		// Level of noise in nodes initial positions:
		const double STD_NOISE_NODE_XYZ = 0.5;
		const double STD_NOISE_NODE_ANG = DEG2RAD(5);

		// Level of noise in edges:
		const double STD_NOISE_EDGE_XYZ = 1e-3;
		const double STD_NOISE_EDGE_ANG = DEG2RAD(0.01);

		for (TNodeID j = 0; j < N_VERTEX; j++)
		{
			static double ang = 2 * M_PI / N_VERTEX;
			const double R = NODES_XY_MAX + 2 * (j % 2 ? 1 : -1);
			CPose2D p(R * cos(ang * j), R * sin(ang * j), ang);

			// Save real pose:
			real_node_poses[j] = p;

			// Copy the nodes to the graph, and add some noise:
			graph.nodes[j] = p;

			const auto noise = CPose3D(
				randomGenerator.drawGaussian1D(0, STD_NOISE_NODE_XYZ),
				randomGenerator.drawGaussian1D(0, STD_NOISE_NODE_XYZ),
				randomGenerator.drawGaussian1D(0, STD_NOISE_NODE_XYZ),
				randomGenerator.drawGaussian1D(0, STD_NOISE_NODE_ANG),
				randomGenerator.drawGaussian1D(0, STD_NOISE_NODE_ANG),
				randomGenerator.drawGaussian1D(0, STD_NOISE_NODE_ANG));
			EdgeAdder<typename my_graph_t::constraint_no_pdf_t>::addNoise(graph.nodes[j], noise);

		}

		// Add some edges
		for (TNodeID i = 0; i < N_VERTEX; i++)
		{
			for (TNodeID j = i + 1; j < N_VERTEX; j++)
			{
				if (real_node_poses[i].distanceTo(real_node_poses[j]) <
					DIST_THRES)
					EdgeAdder<typename my_graph_t::edge_underlying_t>::add<my_graph_t>(i, j, real_node_poses, graph);
			}
		}

		// Cross-links:
		EdgeAdder<typename my_graph_t::edge_underlying_t>::add<my_graph_t>(0, N_VERTEX / 2, real_node_poses, graph);

		// The root node (the origin of coordinates):
		graph.root = TNodeID(0);

		// Add noise to edges & nodes:
		for (auto& edge : graph.edges)
		{
			const auto noise = CPose3D(
				randomGenerator.drawGaussian1D(0, STD_NOISE_EDGE_XYZ),
				randomGenerator.drawGaussian1D(0, STD_NOISE_EDGE_XYZ),
				randomGenerator.drawGaussian1D(0, STD_NOISE_EDGE_XYZ),
				randomGenerator.drawGaussian1D(0, STD_NOISE_EDGE_ANG),
				randomGenerator.drawGaussian1D(0, STD_NOISE_EDGE_ANG),
				randomGenerator.drawGaussian1D(0, STD_NOISE_EDGE_ANG));
			EdgeAdder<typename my_graph_t::edge_underlying_t>::addNoise(edge.second, noise);
		}

	}
};
