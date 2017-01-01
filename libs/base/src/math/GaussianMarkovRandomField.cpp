/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers

#include <mrpt/math/GaussianMarkovRandomField.h>
#include <mrpt/utils/CTicTac.h>

using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace std;

MRPT_TODO("API: add a way to reflect priors/static constraints");

#if EIGEN_VERSION_AT_LEAST(3,1,0) // Requires Eigen>=3.1
#	include <Eigen/SparseCholesky>
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

void GaussianMarkovRandomField::updateEstimation()
{
#if EIGEN_VERSION_AT_LEAST(3,1,0)
	const size_t N = m_g.size();

	m_timelogger.enable(m_enable_profiler);

	m_timelogger.enter("GMRF.build_hessian");

	//------------------
	//  1- HESSIAN
	//------------------
	//Build Sparse Hessian H, from list of triplets (Hprior):
	std::vector<Eigen::Triplet<double> > H_tri;
	H_tri.reserve(m_factors_unary.size() + m_factors_binary.size());

	size_t numActiveObs = 0;
	//Add H_obs
	for (size_t j = 0; j<N; j++)
	{
		//Sum the information of all observations on cell j
		double Lambda_obs_j = 0.0;
		numActiveObs += activeObs[j].size();
		for (std::vector<TobservationGMRF>::const_iterator ito = activeObs[j].begin(); ito != activeObs[j].end(); ++ito)
			Lambda_obs_j += ito->Lambda;

		if (Lambda_obs_j != 0.0)
			H_tri.push_back(Eigen::Triplet<double>(j, j, Lambda_obs_j)); // Duplicated entries (from obs + from the prior) will be summed in setFromTriplets()
	}

	Eigen::SparseMatrix<double> Hsparse(N, N);				// declares a column-major sparse matrix type of float
	Hsparse.setFromTriplets(H_tri.begin(), H_tri.end());

#if 0 // For debug only
	mrpt::math::saveEigenSparseTripletsToFile("H_tri.txt", H_tri);
#endif

	m_timelogger.leave("GMRF.build_hessian");

	if (!numActiveObs) {
		if (m_rfgm_verbose) printf("[CRandomFieldGridMap2D] 0 active observations: skipping map update.\n");
		return;
	}

	m_timelogger.enter("GMRF.build_grad");

	//------------------
	//  2- GRADIENT
	//------------------
	//Reset and Built Gradient Vector
	g.setZero();
	size_t cx = 0;
	size_t cy = 0;
	for (size_t j = 0; j<N; j++)
	{
		//A- Gradient due to Observations
		for (std::vector<TobservationGMRF>::const_iterator ito = activeObs[j].begin(); ito != activeObs[j].end(); ++ito)
			g[j] += ((m_map[j].gmrf_mean - ito->obsValue) * ito->Lambda);

		//B- Gradient due to Prior
		switch (m_mapType)
		{
		case mrGMRF_G:
		{
			const uint16_t Gsize = m_insertOptions_common->GMRF_constraintsSize;
			const uint16_t Gside = mrpt::utils::round((Gsize - 1) / 2);
			//Determine num of columns out of the gridmap
			int outc_left = max(0, int(Gside - cx));
			int outc_right = max(int(0), int(Gside - (m_size_x - cx - 1)));

			//Determine num of rows out of the gridmap
			int outr_down = max(0, int(Gside - (cy)));
			int outr_up = max(int(0), int(Gside - (m_size_y - cy - 1)));

			//Gradients between cell j and all neighbord cells i that have constraints
			for (int kr = -(Gside - outr_down); kr <= (Gside - outr_up); kr++)
			{
				for (int kc = -(Gside - outc_left); kc <= (Gside - outc_right); kc++)
				{
					// get index of cell i
					size_t icx = cx + kc;
					size_t icy = cy + kr;
					size_t i = icx + icy*m_size_x;

					if (j != i)
					{
						if (kr == 0 || kc == 0)      //only vertical and horizontal restrictions/constraints
						{
							g[j] += (m_map[j].gmrf_mean - (m_map[i].gmrf_mean * m_gauss_val[abs(kr + kc) - 1])) * m_insertOptions_common->GMRF_lambdaPrior;
						}
					}
				}
			}
		}
		break;

		case mrGMRF_SD:
		{
			if (this->m_insertOptions_common->GMRF_use_occupancy_information)
			{
				//Consider only cells correlated with the cell j
				std::pair < std::multimap<size_t, size_t>::iterator, std::multimap<size_t, size_t>::iterator > range;
				range = cell_interconnections.equal_range(j);
				while (range.first != range.second)
				{
					size_t cell_i_indx = range.first->second;
					g[j] += (m_map[j].gmrf_mean - m_map[cell_i_indx].gmrf_mean) * m_insertOptions_common->GMRF_lambdaPrior;
					range.first++;
				}

			}
			else
			{
				//Gradient with all 4 neighbours
				if (cx != 0)	//factor with left node
					g[j] += (m_map[j].gmrf_mean - m_map[j - 1].gmrf_mean) * m_insertOptions_common->GMRF_lambdaPrior;

				if (cx != (m_size_x - 1))	//factor with right node
					g[j] += (m_map[j].gmrf_mean - m_map[j + 1].gmrf_mean) * m_insertOptions_common->GMRF_lambdaPrior;

				if (cy != 0)	//factor with benith node
					g[j] += (m_map[j].gmrf_mean - m_map[j - m_size_x].gmrf_mean) * m_insertOptions_common->GMRF_lambdaPrior;

				if (cy != (m_size_y - 1))	//factor with upper node
					g[j] += (m_map[j].gmrf_mean - m_map[j + m_size_x].gmrf_mean) * m_insertOptions_common->GMRF_lambdaPrior;
			}
		}
		break;

		default:
			THROW_EXCEPTION("Gradient estimation error: Unknown method!");
		};

		// Increment j coordinates (row(x), col(y))
		if (++cx >= m_size_x)
		{
			cx = 0;
			cy++;
		}
	}//end-for

	m_timelogger.leave("GMRF.build_grad");
	m_timelogger.enter("GMRF.solve");

	if (m_rfgm_verbose) printf("[CRandomFieldGridMap2D] Solving...\n");
	//Cholesky Factorization of Hessian --> Realmente se hace: chol( P * H * inv(P) )
	Eigen::SimplicialLLT< Eigen::SparseMatrix <double> > solver;
	solver.compute(Hsparse);
	// Solve System:    m = m + H\(-g);
	// Note: we solve for (+g) to avoid creating a temporary "-g", then we'll substract the result in m_inc instead of adding it:


	Eigen::VectorXd m_inc = solver.solve(g);
	if (m_rfgm_verbose) printf("[CRandomFieldGridMap2D] Solved.\n");

	m_timelogger.leave("GMRF.solve");

	Eigen::SparseMatrix<double> Sigma(N, N);								//Variance Matrix
	bool use_variance_perm = true;
	if (!m_insertOptions_common->GMRF_skip_variance)
	{
		m_timelogger.enter("GMRF.variance");

#if 1
		// VARIANCE SIGMA = inv(P) * inv( P*H*inv(P) ) * P
		//Get triangular supperior P*H*inv(P) = UT' * UT = P * R'*R * inv(P)
		MRPT_TODO("Use compressed access instead of coeff() below");

		if (m_rfgm_verbose) printf("[CRandomFieldGridMap2D] Computing variance: U matrix...\n");
		Eigen::SparseMatrix<double> UT = solver.matrixU();
		if (m_rfgm_verbose) printf("[CRandomFieldGridMap2D] Computing variance: U done.\n");
		Sigma.reserve(UT.nonZeros());

		//Apply custom equations to obtain the inverse -> inv( P*H*inv(P) )
		for (int l = N - 1; l >= 0; l--)
		{
			if (m_rfgm_verbose && !(l % 100)) printf("[CRandomFieldGridMap2D] Computing variance %6.02f%%... \r", (100.0*(N - l - 1)) / N);

			//Computes variances in the inferior submatrix of "l"
			double subSigmas = 0.0;
			for (size_t j = l + 1; j<N; j++)
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
							sum += UT.coeff(l, i) * Sigma.coeff(i, j);
						}
					}
					//SUM 2
					for (size_t i = j + 1; i<N; ++i)
					{
						if (UT.coeff(l, i) != 0)
						{
							sum += UT.coeff(l, i) * Sigma.coeff(j, i);
						}
					}
					//Save off-diagonal variance (only Upper triangular)
					Sigma.insert(l, j) = (-sum / UT.coeff(l, l));
					subSigmas += UT.coeff(l, j) * Sigma.coeff(l, j);
				}
			}

			Sigma.insert(l, l) = (1 / UT.coeff(l, l)) * (1 / UT.coeff(l, l) - subSigmas);
		}
#else
		// Naive method: (much slower!)
		Eigen::SparseMatrix<double> I(N, N);
		I.setIdentity();
		Sigma = solver.solve(I);
		use_variance_perm = false;
#endif
		m_timelogger.leave("GMRF.variance");
	}
	m_timelogger.enter("GMRF.copy_to_map");

	// Update Mean-Variance in the base grid class
	for (size_t j = 0; j<N; j++)
	{
		// Recover the diagonal covariance values, undoing the permutation:
		const int idx = use_variance_perm ? (int)solver.permutationP().indices().coeff(j) : (int)j;
		const double variance = Sigma.coeff(idx, idx);

		m_map[j].gmrf_std = std::sqrt(variance);
		m_map[j].gmrf_mean -= m_inc[j]; // "-" because we solved for "+grad" instead of "-grad".

		mrpt::utils::saturate(m_map[j].gmrf_mean, m_insertOptions_common->GMRF_saturate_min, m_insertOptions_common->GMRF_saturate_max);
	}

	// Update Information/Strength of Active Observations
	//---------------------------------------------------------
	if (m_insertOptions_common->GMRF_lambdaObsLoss != 0) {
		for (size_t j = 0; j<activeObs.size(); j++)
		{
			std::vector<TobservationGMRF>::iterator ito = activeObs[j].begin();
			while (ito != activeObs[j].end())
			{
				if (!ito->time_invariant)
				{
					ito->Lambda -= m_insertOptions_common->GMRF_lambdaObsLoss;
					if (ito->Lambda <= 0.0)
						ito = activeObs[j].erase(ito);
					else
						++ito;
				}
				else
					++ito;
			}
		}
	}

	m_timelogger.leave("GMRF.copy_to_map");
#else
	THROW_EXCEPTION("This method requires Eigen 3.1.0 or above")
#endif
