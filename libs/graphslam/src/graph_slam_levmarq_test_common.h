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
#include <mrpt/io/CMemoryStream.h>

using namespace mrpt;
using namespace mrpt::random;
using namespace mrpt::poses;
using namespace mrpt::graphs;
using namespace mrpt::math;
using namespace std;

template <class my_graph_t>
class GraphSlamLevMarqTest
{
   public:
	// adds a new edge to the graph. The edge is annotated with the relative
	// position of the two nodes
	static void addEdge(
		TNodeID from, TNodeID to,
		const typename my_graph_t::global_poses_t& real_poses,
		my_graph_t& graph)
	{
		auto RelativePose =
			real_poses.find(to)->second - real_poses.find(from)->second;

		// Add information matrix if present:
		if constexpr (my_graph_t::edge_t::is_PDF())
		{
			const auto N = my_graph_t::edge_t::state_length;
			using InfMat = mrpt::math::CMatrixFixedNumeric<double, N, N>;
			auto mat = getRandomGenerator()
						   .drawDefinitePositiveMatrix<InfMat, Eigen::VectorXd>(
							   N, 2.0 /*std*/, 1.0 /*diagonal offset*/);
			graph.insertEdge(
				from, to, typename my_graph_t::edge_t(RelativePose, mat));
		}
		else
		{
			graph.insertEdge(from, to, RelativePose);
		}
	}

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
		const double STD_NOISE_EDGE_XYZ = 0;  // 0.01;
		const double STD_NOISE_EDGE_ANG = 0;  // DEG2RAD(0.1);

		for (TNodeID j = 0; j < N_VERTEX; j++)
		{
			static double ang = 2 * M_PI / N_VERTEX;
			const double R = NODES_XY_MAX + 2 * (j % 2 ? 1 : -1);
			CPose2D p(R * cos(ang * j), R * sin(ang * j), ang);

			// Save real pose:
			real_node_poses[j] = p;

			// Copy the nodes to the graph, and add some noise:
			graph.nodes[j] = p;
		}

		// Add some edges
		for (TNodeID i = 0; i < N_VERTEX; i++)
		{
			for (TNodeID j = i + 1; j < N_VERTEX; j++)
			{
				if (real_node_poses[i].distanceTo(real_node_poses[j]) <
					DIST_THRES)
					addEdge(i, j, real_node_poses, graph);
			}
		}

		// Cross-links:
		addEdge(0, N_VERTEX / 2, real_node_poses, graph);

		// The root node (the origin of coordinates):
		graph.root = TNodeID(0);

		// This is the ground truth graph (make a copy for later use):
		const my_graph_t graph_GT = graph;

		// Add noise to edges & nodes:
		for (auto& edge : graph.edges)
		{
			// edge_t::type_value: for edge_t a plain pose, type_value is the
			// edge_t itself. For poses+uncertainty, it is the underlying pose
			// type.
			if constexpr (my_graph_t::edge_t::type_value::size() == 6)
			{
				const auto noise = CPose3D(
					getRandomGenerator().drawGaussian1D(0, STD_NOISE_EDGE_XYZ),
					getRandomGenerator().drawGaussian1D(0, STD_NOISE_EDGE_XYZ),
					getRandomGenerator().drawGaussian1D(0, STD_NOISE_EDGE_XYZ),
					getRandomGenerator().drawGaussian1D(0, STD_NOISE_EDGE_ANG),
					getRandomGenerator().drawGaussian1D(0, STD_NOISE_EDGE_ANG),
					getRandomGenerator().drawGaussian1D(0, STD_NOISE_EDGE_ANG));

				if constexpr (my_graph_t::edge_t::is_PDF())
				{
					const auto N = my_graph_t::edge_t::state_length;
					using InfMat =
						mrpt::math::CMatrixFixedNumeric<double, N, N>;
					InfMat mat;
					mat.setIdentity();
					edge.second += typename my_graph_t::edge_t(noise, mat);
				}
				else
				{
					edge.second += typename my_graph_t::edge_t(noise);
				}
			}
			else
			{
				const auto noise = CPose2D(
					getRandomGenerator().drawGaussian1D(0, STD_NOISE_EDGE_XYZ),
					getRandomGenerator().drawGaussian1D(0, STD_NOISE_EDGE_XYZ),
					getRandomGenerator().drawGaussian1D(0, STD_NOISE_EDGE_ANG));
				if constexpr (my_graph_t::edge_t::is_PDF())
				{
					const auto N = my_graph_t::edge_t::state_length;
					using InfMat =
						mrpt::math::CMatrixFixedNumeric<double, N, N>;
					InfMat mat;
					mat.setIdentity();
					edge.second += typename my_graph_t::edge_t(noise, mat);
				}
				else
				{
					edge.second += typename my_graph_t::edge_t(noise);
				}
			}
		}

		for (auto itNode = graph.nodes.begin(); itNode != graph.nodes.end();
			 ++itNode)
			if (itNode->first != graph.root)
				itNode
					->second += typename my_graph_t::edge_t::type_value(CPose3D(
					getRandomGenerator().drawGaussian1D(0, STD_NOISE_NODE_XYZ),
					getRandomGenerator().drawGaussian1D(0, STD_NOISE_NODE_XYZ),
					getRandomGenerator().drawGaussian1D(0, STD_NOISE_NODE_XYZ),
					getRandomGenerator().drawGaussian1D(0, STD_NOISE_NODE_ANG),
					getRandomGenerator().drawGaussian1D(0, STD_NOISE_NODE_ANG),
					getRandomGenerator().drawGaussian1D(
						0, STD_NOISE_NODE_ANG)));
	}
};
