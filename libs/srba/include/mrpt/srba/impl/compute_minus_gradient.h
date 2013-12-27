/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#pragma once

namespace mrpt { namespace srba {

using namespace mrpt;
using namespace mrpt::math;
using namespace std;

/*******************************************
      compute_minus_gradient

	    grad = J^t * (h(x)-z)
 *******************************************/
template <class KF2KF_POSE_TYPE,class LM_TYPE,class OBS_TYPE,class RBA_OPTIONS>
void RbaEngine<KF2KF_POSE_TYPE,LM_TYPE,OBS_TYPE,RBA_OPTIONS>::compute_minus_gradient(
	mrpt::vector_double & minus_grad,
	const std::vector<typename TSparseBlocksJacobians_dh_dAp::col_t*> & sparse_jacobs_Ap,
	const std::vector<typename TSparseBlocksJacobians_dh_df::col_t*> & sparse_jacobs_f,
	const vector_residuals_t  & residuals,
	const std::map<size_t,size_t> &obs_global_idx2residual_idx
	) const
{
	// Problem dimensions:
	const size_t POSE_DIMS = KF2KF_POSE_TYPE::REL_POSE_DIMS;
	const size_t LM_DIMS   = LM_TYPE::LM_DIMS;

	const size_t nUnknowns_k2k = sparse_jacobs_Ap.size();
	const size_t nUnknowns_k2f = sparse_jacobs_f.size();

	const size_t idx_start_f = POSE_DIMS*nUnknowns_k2k;
	const size_t nUnknowns_scalars = POSE_DIMS*nUnknowns_k2k + LM_DIMS*nUnknowns_k2f;

	if (static_cast<size_t>(minus_grad.size())!=nUnknowns_scalars)
		minus_grad.resize(nUnknowns_scalars);

	//size_t running_idx_obs=0; // for the precomputed "sequential_obs_indices"

	// grad_Ap:
	for (size_t i=0;i<nUnknowns_k2k;i++)
	{
		const typename TSparseBlocksJacobians_dh_dAp::col_t & col_i = *sparse_jacobs_Ap[i];

		array_pose_t accum_g_i;
		accum_g_i.zeros();

		for (typename TSparseBlocksJacobians_dh_dAp::col_t::const_iterator itJ = col_i.begin();itJ != col_i.end();++itJ)
		{
			//const size_t resid_idx = sequential_obs_indices[running_idx_obs++];
			const size_t obs_idx = itJ->first;
			map<size_t,size_t>::const_iterator it_obs = obs_global_idx2residual_idx.find(obs_idx);
			ASSERT_(it_obs!=obs_global_idx2residual_idx.end())
			const size_t resid_idx = it_obs->second;

			// Accumulate sub-gradient: // g += J^t * \Lambda * residual 
			RBA_OPTIONS::obs_noise_matrix_t::template accum_Jtr(accum_g_i, itJ->second.num, residuals[ resid_idx ], obs_idx, this->parameters.obs_noise );
		}
		// Do scaling (if applicable):
		RBA_OPTIONS::obs_noise_matrix_t::template scale_Jtr(accum_g_i, this->parameters.obs_noise );

		minus_grad.block<POSE_DIMS,1>(i*POSE_DIMS,0) = accum_g_i;
	}
	// grad_Af:
	for (size_t i=0;i<nUnknowns_k2f;i++)
	{
		const typename TSparseBlocksJacobians_dh_df::col_t & col_i = *sparse_jacobs_f[i];

		array_landmark_t accum_g_i;
		accum_g_i.zeros();

		for (typename TSparseBlocksJacobians_dh_df::col_t::const_iterator itJ = col_i.begin();itJ != col_i.end();++itJ)
		{
			//const size_t resid_idx = sequential_obs_indices[running_idx_obs++];
			const size_t obs_idx = itJ->first;
			map<size_t,size_t>::const_iterator it_obs = obs_global_idx2residual_idx.find(obs_idx);
			ASSERT_(it_obs!=obs_global_idx2residual_idx.end())
			const size_t resid_idx = it_obs->second;

			// Accumulate sub-gradient: // g += J^t * \Lambda * residual 
			RBA_OPTIONS::obs_noise_matrix_t::template accum_Jtr(accum_g_i, itJ->second.num, residuals[ resid_idx ], obs_idx, this->parameters.obs_noise );
		}
		// Do scaling (if applicable):
		RBA_OPTIONS::obs_noise_matrix_t::template scale_Jtr(accum_g_i, this->parameters.obs_noise );

		minus_grad.block<LM_DIMS,1>(idx_start_f+i*LM_DIMS,0) = accum_g_i;
	}
}

} }  // end namespaces
