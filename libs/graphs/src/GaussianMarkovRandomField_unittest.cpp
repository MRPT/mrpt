/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */


#include <mrpt/graphs/GaussianMarkovRandomField.h>
#include <gtest/gtest.h>

using namespace mrpt;
using namespace mrpt::graphs;
using namespace std;

#if EIGEN_VERSION_AT_LEAST(3,1,0) // Requires Eigen>=3.1

TEST(GaussianMarkovRandomField, SimpleOps)
{
	const size_t N = 4;
	vector<double> my_map(N, .0);
	
	struct MySimpleUnaryEdge : public GaussianMarkovRandomField::UnaryFactorVirtualBase
	{
		MySimpleUnaryEdge(vector<double> &parent, size_t nodeid, double observation, double information) : 
			m_parent(parent),
			m_observation(observation),
			m_information(information)
		{
			this->node_id = nodeid;
		}

		double evaluateResidual() const override
		{
			return m_parent[node_id] - m_observation;
		}
		double getInformation() const override
		{
			return m_information;
		}
		void evalJacobian(double &dr_dx) const override
		{
			dr_dx = 1.0;
		}

	protected:
		vector<double> &m_parent;
		double m_observation, m_information;
	};
	
	GaussianMarkovRandomField  gmrf;

	gmrf.initialize(N);

	std::deque<MySimpleUnaryEdge>  edges1;

	edges1.push_back(MySimpleUnaryEdge(my_map, 0, 1.0, 4.0));
	edges1.push_back(MySimpleUnaryEdge(my_map, 1, 5.0, 4.0));
	edges1.push_back(MySimpleUnaryEdge(my_map, 2, 3.0, 4.0));
	edges1.push_back(MySimpleUnaryEdge(my_map, 3, 2.0, 16.0));

	for (const auto &e:edges1)
		gmrf.addConstraint(e);

	Eigen::VectorXd              x_incr;
	Eigen::SparseMatrix<double>  x_cov;
	gmrf.updateEstimation(x_incr, &x_cov);


}

#endif // Eigen>=3.1
