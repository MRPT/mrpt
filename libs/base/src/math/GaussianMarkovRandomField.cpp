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

GaussianMarkovRandomField::GaussianMarkovRandomField() :
	COutputLogger("GaussianMarkovRandomField")
{
}

void GaussianMarkovRandomField::init_priors(const InitPriorInformer &params)
{
	CTicTac tictac;
	tictac.Tic();

	MRPT_LOG_DEBUG("[init_priors]: Generating priors...");

	const size_t N = params.getNodeCount();

	//Determine the number of Initial constraints --> L+M
	m_nPriorFactors = params.getPriorFactorsCount(); // L
	m_nObsFactors = 0;   // M
	m_nFactors = m_nPriorFactors + m_nObsFactors;

	MRPT_LOG_DEBUG_STREAM << "[init_priors]: Generating " << m_nFactors << " cell constraints for a graph size N=" << N << std::endl;

	//Reset the vector of maps (Hessian_vm), the gradient, and the vector of active observations
#if EIGEN_VERSION_AT_LEAST(3,1,0)
	m_H_prior.clear();
	m_H_prior.reserve(m_nPriorFactors);
#endif
	m_g.resize(N);			//Initially the gradient is all 0
	m_g.fill(0.0);
	m_activeObs.resize(N);	//No initial Observations

	// Load default values:
	{
		// Avoid the costly % and / operations:
		size_t cx = 0; // ( j % m_size_x );		// [0, m_size_x-1]
		size_t cy = 0; // ( j / m_size_x );		// [0, m_size_y-1]
		size_t count = 0;
		for (size_t j = 0; j<N; j++)
		{
			//Determine the Number of restrictions of the current cell j
			//Determine num of columns out of the gridmap
			int outc_left = max(0, int(Gside - cx));
			int outc_right = max(int(0), int(Gside - (m_size_x - cx - 1)));
			int outc = outc_left + outc_right;
			//Determine num of rows out of the gridmap
			int outr_down = max(0, int(Gside - (cy)));
			int outr_up = max(int(0), int(Gside - (m_size_y - cy - 1)));
			int outr = outr_up + outr_down;

			size_t nConsFixed_j = (Gsize - outc - 1) + (Gsize - outr - 1);   //only vertical and horizontal restrictions

																			 //Set constraints of cell j with all neighbord cells i
			for (int kr = -(Gside - outr_down); kr <= (Gside - outr_up); kr++)
			{
				for (int kc = -(Gside - outc_left); kc <= (Gside - outc_right); kc++)
				{
					// get index of cell i
					size_t icx = cx + kc;
					size_t icy = cy + kr;
					size_t i = icx + icy*m_size_x;

					if (j == i)
					{
						//H_ii = N constraints * Lambda_cell * (J_ij^2 +1)
#if EIGEN_VERSION_AT_LEAST(3,1,0)
						//std::pair<size_t,float> Hentry (j , nConsFixed_j * m_insertOptions_common->GMRF_lambdaPrior * (square(1.0/m_gauss_val[abs(kr+kc)]) + 1) );
						//H_vm.at(i).insert(Hentry);

						Eigen::Triplet<double> Hentry(i, j, nConsFixed_j * m_insertOptions_common->GMRF_lambdaPrior * (square(1.0 / m_gauss_val[abs(kr + kc)]) + 1));
						m_H_prior.push_back(Hentry);
#endif
					}
					else
					{
						if (kr == 0 || kc == 0)      //only vertical and horizontal restrictions/constraints
						{
							// H_ji = 2 * Lambda_cell * J_ij
#if EIGEN_VERSION_AT_LEAST(3,1,0)
							Eigen::Triplet<double> Hentry(i, j, -2 * m_insertOptions_common->GMRF_lambdaPrior * 1 / m_gauss_val[abs(kr + kc) - 1]);
							m_H_prior.push_back(Hentry);
#endif
							//g_j = [m(j) - alpha*m(i) ]* lambda
							m_g[j] += (m_map[j].gmrf_mean - m_map[i].gmrf_mean) * m_insertOptions_common->GMRF_lambdaPrior;

							count++;
						}
					}
				}
			}

			// Increment (row(x), col(y))
			if (++cx >= m_size_x)
			{
				cx = 0;
				cy++;
			}
		} // end for "j"
	}

	if (m_rfgm_verbose) cout << "		Ready in: " << tictac.Tac() << "s" << endl;
}
