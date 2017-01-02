/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "graphs-precomp.h"  // Precompiled headers

#include <mrpt/graphs/GaussianMarkovRandomField.h>
#include <mrpt/utils/CTicTac.h>

using namespace mrpt;
using namespace mrpt::graphs;
using namespace mrpt::utils;
using namespace std;

MRPT_TODO("API: add a way to reflect priors/static constraints");

#if EIGEN_VERSION_AT_LEAST(3,1,0) // Requires Eigen>=3.1
//#	include <Eigen/SparseCholesky>
#	include <Eigen/SparseQR>
#endif

GaussianMarkovRandomField::GaussianMarkovRandomField() :
	COutputLogger("GMRF"),
	m_enable_profiler(true)
{
}

void GaussianMarkovRandomField::clear()
{
	MRPT_LOG_DEBUG("clear() called");

	m_g.resize(0);
	m_factors_unary.clear();
	m_factors_binary.clear();
}

void GaussianMarkovRandomField::initialize(const size_t nodeCount)
{
	MRPT_LOG_DEBUG_STREAM << "initialize() called, nodeCount=" << nodeCount;

	m_g.resize(nodeCount); //Initially the gradient is all 0
	m_g.fill(0.0);
}

void GaussianMarkovRandomField::addConstraint(const UnaryFactorVirtualBase &c)
{
	m_factors_unary.push_back( &c );
}
void GaussianMarkovRandomField::addConstraint(const BinaryFactorVirtualBase &c)
{
	m_factors_binary.push_back(&c);
}

/* Method:
  (\Sigma)^{-1/2) *  d( h(x) )/d( x )  * x_incr = - (\Sigma)^{-1/2) * r(x)
  ===================================            ========================
              =A                                           =b

   A * x_incr = b         --> SparseQR.
*/
void GaussianMarkovRandomField::updateEstimation(
	Eigen::VectorXd & solved_x_inc,                 //!< Output increment of the current estimate. Caller must add this vector to current state vector to obtain the optimal estimation.
	Eigen::SparseMatrix<double> *solved_covariance  //!< If !=NULL, the covariance of the estimate will be stored here.
)
{
	m_timelogger.enable(m_enable_profiler);

#if EIGEN_VERSION_AT_LEAST(3,1,0)

	// Number of vertices:
	const size_t n = m_g.size();
	solved_x_inc.setZero(n);

	// Number of edges:
	const size_t m1 = m_factors_unary.size(), m2 = m_factors_binary.size();
	const size_t m = m1 + m2;

	// Build Ax=b
	// -----------------------
	m_timelogger.enter("GMRF.build_A_tri");

	std::vector<Eigen::Triplet<double> > A_tri;
	A_tri.reserve(m1 + 2 * m2);

	m_g.setZero();
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
		m_g[edge_counter] -= w*e->evaluateResidual();

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
		m_g[edge_counter] -= w*e->evaluateResidual();

		++edge_counter;
	}
	m_timelogger.leave("GMRF.build_A_tri");

	// Compress sparse
	// -----------------------
	Eigen::SparseMatrix<double> A(m, n);
	{
		mrpt::utils::CTimeLoggerEntry tle(m_timelogger, "GMRF.build_A_compress");

		A.setFromTriplets(A_tri.begin(), A_tri.end());
#if 0 // For debug only
		mrpt::math::saveEigenSparseTripletsToFile("H_tri.txt", H_tri);
#endif
		A.makeCompressed();
	}

	// Solve increment
	// -----------------------
	Eigen::SparseQR<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int> > solver;
	{
		mrpt::utils::CTimeLoggerEntry tle(m_timelogger, "GMRF.solve");

		solver.compute(A);
		solved_x_inc = solver.solve(m_g);
	}

	// Recover covariance
	// -----------------------
	bool use_variance_perm = true;
	if (solved_covariance)
	{
		mrpt::utils::CTimeLoggerEntry tle(m_timelogger, "GMRF.variance");

#if 1
		// VARIANCE SIGMA = inv(P) * inv( P*H*inv(P) ) * P
		//Get triangular supperior P*H*inv(P) = UT' * UT = P * R'*R * inv(P)
		MRPT_TODO("Use compressed access instead of coeff() below");

		MRPT_TODO("UT=R... check!");
		Eigen::SparseMatrix<double> UT = solver.matrixR();
		*solved_covariance = Eigen::SparseMatrix<double>(n,n);
		solved_covariance->reserve(UT.nonZeros());

		//Apply custom equations to obtain the inverse -> inv( P*H*inv(P) )
		for (int l = n - 1; l >= 0; l--)
		{
			if (!(l % 100))
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
							sum += UT.coeff(l, i) * solved_covariance->coeff(i, j);
						}
					}
					//SUM 2
					for (size_t i = j + 1; i < n; ++i)
					{
						if (UT.coeff(l, i) != 0)
						{
							sum += UT.coeff(l, i) * solved_covariance->coeff(j, i);
						}
					}
					//Save off-diagonal variance (only Upper triangular)
					solved_covariance->insert(l, j) = (-sum / UT.coeff(l, l));
					subSigmas += UT.coeff(l, j) * solved_covariance->coeff(l, j);
				}
			}

			solved_covariance->insert(l, l) = (1 / UT.coeff(l, l)) * (1 / UT.coeff(l, l) - subSigmas);
		}
#else
		// Naive method: (much slower!)
		Eigen::SparseMatrix<double> I(n, n);
		I.setIdentity();
		*solved_covariance = solver.solve(I);
		use_variance_perm = false;
#endif
	}

#else
	THROW_EXCEPTION("This method requires Eigen 3.1.0 or above")
#endif
}
