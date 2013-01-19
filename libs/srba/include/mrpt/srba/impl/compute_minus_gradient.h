/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
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
void RBA_Problem<KF2KF_POSE_TYPE,LM_TYPE,OBS_TYPE,RBA_OPTIONS>::compute_minus_gradient(
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
			map<size_t,size_t>::const_iterator it_obs = obs_global_idx2residual_idx.find(itJ->first);
			ASSERT_(it_obs!=obs_global_idx2residual_idx.end())
			const size_t resid_idx = it_obs->second;

			accum_g_i.noalias() += itJ->second.num.transpose() * residuals[ resid_idx ];
		}

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
			map<size_t,size_t>::const_iterator it_obs = obs_global_idx2residual_idx.find(itJ->first);
			ASSERT_(it_obs!=obs_global_idx2residual_idx.end())
			const size_t resid_idx = it_obs->second;

			accum_g_i.noalias() += itJ->second.num.transpose() * residuals[ resid_idx ];
		}

		minus_grad.block<LM_DIMS,1>(idx_start_f+i*LM_DIMS,0) = accum_g_i;
	}
}

} }  // end namespaces
