/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "graphs-precomp.h"  // Precompiled headers

#include <mrpt/graphs/ScalarFactorGraph.h>
#include <mrpt/utils/CTicTac.h>

using namespace mrpt;
using namespace mrpt::graphs;
using namespace mrpt::utils;
using namespace std;

#if EIGEN_VERSION_AT_LEAST(3,1,0) // Requires Eigen>=3.1
#	include <Eigen/SparseCore>
#	include <Eigen/SparseQR>
#endif


ScalarFactorGraph::FactorBase::~FactorBase()
{
}

ScalarFactorGraph::ScalarFactorGraph() :
	COutputLogger("GMRF"),
	m_enable_profiler(false)
{
}

void ScalarFactorGraph::clear()
{
	MRPT_LOG_DEBUG("clear() called");

	m_numNodes = 0;
	m_factors_unary.clear();
	m_factors_binary.clear();
}

void ScalarFactorGraph::initialize(const size_t nodeCount)
{
	MRPT_LOG_DEBUG_STREAM << "initialize() called, nodeCount=" << nodeCount;

	m_numNodes = nodeCount;
}

void ScalarFactorGraph::addConstraint(const UnaryFactorVirtualBase &c)
{
	m_factors_unary.push_back( &c );
}
void ScalarFactorGraph::addConstraint(const BinaryFactorVirtualBase &c)
{
	m_factors_binary.push_back(&c);
}

bool ScalarFactorGraph::eraseConstraint(const FactorBase &c)
{
	{
		auto it = std::find(m_factors_unary.begin(), m_factors_unary.end(), &c);
		if (it != m_factors_unary.end())
		{
			m_factors_unary.erase(it);
			return true;
		}
	}
	{
		auto it = std::find(m_factors_binary.begin(), m_factors_binary.end(), &c);
		if (it != m_factors_binary.end())
		{
			m_factors_binary.erase(it);
			return true;
		}
	}
	return false;
}


/* Method:
  (\Sigma)^{-1/2) *  d( h(x) )/d( x )  * x_incr = - (\Sigma)^{-1/2) * r(x)
  ===================================            ========================
              =A                                           =b

   A * x_incr = b         --> SparseQR.
*/
void ScalarFactorGraph::updateEstimation(
	Eigen::VectorXd & solved_x_inc,                 //!< Output increment of the current estimate. Caller must add this vector to current state vector to obtain the optimal estimation.
	Eigen::VectorXd * solved_variances  //!< If !=NULL, the covariance of the estimate will be stored here.
)
{
	ASSERTMSG_(m_numNodes>0, "numNodes=0. Have you called initialize()?");

	m_timelogger.enable(m_enable_profiler);

#if EIGEN_VERSION_AT_LEAST(3,1,0)

	// Number of vertices:
	const size_t n = m_numNodes;
	solved_x_inc.setZero(n);

	// Number of edges:
	const size_t m1 = m_factors_unary.size(), m2 = m_factors_binary.size();
	const size_t m = m1 + m2;

	// Build Ax=b
	// -----------------------
	m_timelogger.enter("GMRF.build_A_tri");

	std::vector<Eigen::Triplet<double> > A_tri;
	A_tri.reserve(m1 + 2 * m2);

	Eigen::VectorXd g; // Error vector

	g.setZero(m);
	int edge_counter = 0;
	for (const auto &e : m_factors_unary)
	{
		ASSERT_(e != nullptr);
		// Jacob:
		const double w = std::sqrt(e->getInformation());
		double dr_dx;
		e->evalJacobian(dr_dx);
		const int node_id = e->node_id;
		A_tri.push_back(Eigen::Triplet<double>(edge_counter, node_id, w*dr_dx));
		// gradient:
		g[edge_counter] -= w*e->evaluateResidual();

		++edge_counter;
	}
	for (const auto &e : m_factors_binary)
	{
		ASSERT_(e != nullptr);
		// Jacob:
		const double w = std::sqrt(e->getInformation());
		double dr_dxi, dr_dxj;
		e->evalJacobian(dr_dxi, dr_dxj);
		const int node_id_i = e->node_id_i, node_id_j = e->node_id_j;
		A_tri.push_back(Eigen::Triplet<double>(edge_counter, node_id_i, w*dr_dxi));
		A_tri.push_back(Eigen::Triplet<double>(edge_counter, node_id_j, w*dr_dxj));
		// gradient:
		g[edge_counter] -= w*e->evaluateResidual();

		++edge_counter;
	}
	m_timelogger.leave("GMRF.build_A_tri");

	// Compress sparse
	// -----------------------
	Eigen::SparseMatrix<double> A(m, n);
	{
		mrpt::utils::CTimeLoggerEntry tle(m_timelogger, "GMRF.build_A_compress");

		A.setFromTriplets(A_tri.begin(), A_tri.end());
		A.makeCompressed();
	}

	// Solve increment
	// -----------------------
	Eigen::SparseQR<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int> > solver;
	{
		mrpt::utils::CTimeLoggerEntry tle(m_timelogger, "GMRF.solve");

		solver.compute(A);
		solved_x_inc = solver.solve(g);
	}

	// Recover covariance
	// -----------------------
	if (solved_variances)
	{
		mrpt::utils::CTimeLoggerEntry tle(m_timelogger, "GMRF.variance");

		solved_variances->resize(n);

		// VARIANCE SIGMA = inv(P) * inv( P*H*inv(P) ) * P
		//Get triangular supperior P*H*inv(P) = UT' * UT = P * R'*R * inv(P)
		// (QR factor: Use UT=R)

		MRPT_TODO("Use compressed access instead of coeff() below");

		Eigen::SparseMatrix<double> UT = solver.matrixR();
		Eigen::SparseMatrix<double> solved_covariance(n,n);
		solved_covariance.reserve(UT.nonZeros());

		//Apply custom equations to obtain the inverse -> inv( P*H*inv(P) )
		const int show_progress_steps = std::max(int(20), int(n / 20));
		for (int l = n - 1; l >= 0; l--)
		{
			if (!(l % show_progress_steps))
				MRPT_LOG_DEBUG_FMT("Computing variance %6.02f%%... \r", (100.0*(n - l - 1)) / n);

			//Computes variances in the inferior submatrix of "l"
			double subSigmas = 0.0;
			for (size_t j = l + 1; j < n; j++)
			{
				if (UT.coeff(l, j) != 0)
				{
					//Compute off-diagonal variances Sigma(j,l) = Sigma(l,j);

					//SUM 1
					double sum = 0.0;
					for (size_t i = l + 1; i <= j; i++)
					{
						if (UT.coeff(l, i) != 0)
						{
							sum += UT.coeff(l, i) * solved_covariance.coeff(i, j);
						}
					}
					//SUM 2
					for (size_t i = j + 1; i < n; ++i)
					{
						if (UT.coeff(l, i) != 0)
						{
							sum += UT.coeff(l, i) * solved_covariance.coeff(j, i);
						}
					}
					//Save off-diagonal variance (only Upper triangular)
					solved_covariance.insert(l, j) = (-sum / UT.coeff(l, l));
					subSigmas += UT.coeff(l, j) * solved_covariance.coeff(l, j);
				}
			}

			solved_covariance.insert(l, l) = (1 / UT.coeff(l, l)) * (1 / UT.coeff(l, l) - subSigmas);
		}

		MRPT_LOG_DEBUG_FMT("Computing variance %6.02f%%... \r", 100.0);

		for (unsigned int i = 0; i < n; i++)
		{
			const int idx = (int)solver.colsPermutation().indices().coeff(i);
			const double variance = solved_covariance.coeff(i, i);
			(*solved_variances)[idx] = variance;
		}

	} // end calc variances

#else
	THROW_EXCEPTION("This method requires Eigen 3.1.0 or above")
#endif
}
