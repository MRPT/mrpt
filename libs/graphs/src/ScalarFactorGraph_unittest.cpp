/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */


#include <mrpt/graphs/ScalarFactorGraph.h>
#include <gtest/gtest.h>

using namespace mrpt;
using namespace mrpt::graphs;
using namespace std;

#if EIGEN_VERSION_AT_LEAST(3,1,0) // Requires Eigen>=3.1

struct MySimpleUnaryEdge : public ScalarFactorGraph::UnaryFactorVirtualBase
{
	MySimpleUnaryEdge(vector<double> &parent, size_t nodeid, double observation, double information) :
		m_parent(parent),
		m_observation(observation),
		m_information(information)
	{
		this->node_id = nodeid;
	}

	double evaluateResidual() const MRPT_OVERRIDE
	{
		return m_parent[node_id] - m_observation;
	}
	double getInformation() const MRPT_OVERRIDE
	{
		return m_information;
	}
	void evalJacobian(double &dr_dx) const MRPT_OVERRIDE
	{
		dr_dx = 1.0;
	}

protected:
	vector<double> &m_parent;
	double m_observation, m_information;
};

struct MySimpleBinaryEdge : public ScalarFactorGraph::BinaryFactorVirtualBase
{
	MySimpleBinaryEdge(vector<double> &parent, size_t nodeid_i, size_t nodeid_j, double information) :
		m_parent(parent),
		m_information(information)
	{
		this->node_id_i = nodeid_i;
		this->node_id_j = nodeid_j;
	}

	double evaluateResidual() const MRPT_OVERRIDE
	{
		return m_parent[node_id_i] - m_parent[node_id_j];
	}
	double getInformation() const MRPT_OVERRIDE
	{
		return m_information;
	}
	void evalJacobian(double &dr_dx_i, double &dr_dx_j ) const MRPT_OVERRIDE
	{
		dr_dx_i = +1.0;
		dr_dx_j = -1.0;
	}

protected:
	vector<double> &m_parent;
	double m_information;
};

TEST(ScalarFactorGraph, MiniMRF_UnaryEdges)
{
	const size_t N = 4;
	vector<double> my_map(N, .0);

	ScalarFactorGraph  gmrf;
	gmrf.enableProfiler(false);

	gmrf.initialize(N);

	std::deque<MySimpleUnaryEdge>  edges1;

	edges1.push_back(MySimpleUnaryEdge(my_map, 0, 1.0, 4.0));
	edges1.push_back(MySimpleUnaryEdge(my_map, 1, 5.0, 4.0));
	edges1.push_back(MySimpleUnaryEdge(my_map, 2, 3.0, 4.0));
	edges1.push_back(MySimpleUnaryEdge(my_map, 3, 2.0, 16.0));

	for (const auto &e:edges1)
		gmrf.addConstraint(e);

	// Test 1:
	// --------------
	{
		my_map.assign(N, .0);

		Eigen::VectorXd x_incr, x_var;
		gmrf.updateEstimation(x_incr, &x_var);

		EXPECT_NEAR(x_incr[0], 1.0, 1e-9);
		EXPECT_NEAR(x_incr[1], 5.0, 1e-9);
		EXPECT_NEAR(x_incr[2], 3.0, 1e-9);
		EXPECT_NEAR(x_incr[3], 2.0, 1e-9);

		EXPECT_NEAR(x_var[0], 1.0 / 4.0, 1e-9);
		EXPECT_NEAR(x_var[1], 1.0 / 4.0, 1e-9);
		EXPECT_NEAR(x_var[2], 1.0 / 4.0, 1e-9);
		EXPECT_NEAR(x_var[3], 1.0 / 16.0, 1e-9);
	}

	// Test 2:
	// --------------
	{
		my_map.assign(N, .0);

		// Add new edge:
		edges1.push_back(MySimpleUnaryEdge(my_map, 0, 4.0, 2.0));
		gmrf.addConstraint(*edges1.rbegin());

		Eigen::VectorXd x_incr, x_var;
		gmrf.updateEstimation(x_incr, &x_var);

		EXPECT_NEAR(x_incr[0], 2.0, 1e-9);
		EXPECT_NEAR(x_incr[1], 5.0, 1e-9);
		EXPECT_NEAR(x_incr[2], 3.0, 1e-9);
		EXPECT_NEAR(x_incr[3], 2.0, 1e-9);

		EXPECT_NEAR(x_var[0], 1.0/(4.0 + 2.0), 1e-9);
		EXPECT_NEAR(x_var[1], 1.0 / 4.0, 1e-9);
		EXPECT_NEAR(x_var[2], 1.0 / 4.0, 1e-9);
		EXPECT_NEAR(x_var[3], 1.0 / 16.0, 1e-9);
	}
}

TEST(ScalarFactorGraph, MiniMRF_BinaryEdges)
{
	const size_t N = 4;
	vector<double> my_map(N, .0);

	ScalarFactorGraph  gmrf;
	gmrf.enableProfiler(false);

	gmrf.initialize(N);

	// Edge to assign a value to node 0:
	MySimpleUnaryEdge edge_val0(my_map, 0, 1.0 /*value*/, 1.0 /*information*/);
	gmrf.addConstraint(edge_val0);

	MySimpleBinaryEdge edge_01(my_map, 0, 1, .1);
	gmrf.addConstraint(edge_01);
	MySimpleBinaryEdge edge_12(my_map, 1, 2, .1);
	gmrf.addConstraint(edge_12);
	MySimpleBinaryEdge edge_23(my_map, 2, 3, .1);
	gmrf.addConstraint(edge_23);
	MySimpleBinaryEdge edge_30(my_map, 3, 0, .1);
	gmrf.addConstraint(edge_30);

	// Test 1:
	// --------------
	{
		my_map.assign(N, .0);

		Eigen::VectorXd x_incr, x_var;
		gmrf.updateEstimation(x_incr, &x_var);

		EXPECT_NEAR(x_incr[0], 1.0, 1e-6);
		EXPECT_NEAR(x_incr[1], 1.0, 1e-6);
		EXPECT_NEAR(x_incr[2], 1.0, 1e-6);
		EXPECT_NEAR(x_incr[3], 1.0, 1e-6);

		EXPECT_NEAR(x_var[0], 1.0 , 1e-6);
		
		EXPECT_GT(x_var[1], x_var[0]);
		EXPECT_GT(x_var[3], x_var[0]);

		EXPECT_GT(x_var[2], x_var[1]);
		EXPECT_GT(x_var[2], x_var[3]);
	}
}

#endif // Eigen>=3.1
