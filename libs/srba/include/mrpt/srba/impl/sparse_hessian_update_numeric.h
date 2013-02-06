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

/** Rebuild the Hessian symbolic information from the internal pointers to blocks of Jacobians.
	*  Only the upper triangle is filled-in (all what is needed for Cholesky) for square Hessians, in whole for rectangular ones (it depends on the symbolic decomposition, done elsewhere).
	* \tparam SPARSEBLOCKHESSIAN can be: TSparseBlocksHessian_6x6, TSparseBlocksHessian_3x3 or TSparseBlocksHessian_6x3
	* \return The number of Jacobian multiplications skipped due to its observation being marked as "invalid"
	*/
template <class KF2KF_POSE_TYPE,class LM_TYPE,class OBS_TYPE,class RBA_OPTIONS>
template <class SPARSEBLOCKHESSIAN>
size_t RBA_Problem<KF2KF_POSE_TYPE,LM_TYPE,OBS_TYPE,RBA_OPTIONS>::sparse_hessian_update_numeric( SPARSEBLOCKHESSIAN & H ) const
{
	size_t nInvalid = 0;
	const size_t nUnknowns = H.getColCount();
	for (size_t i=0;i<nUnknowns;i++)
	{
		typename SPARSEBLOCKHESSIAN::col_t & col = H.getCol(i);

		for (typename SPARSEBLOCKHESSIAN::col_t::iterator it=col.begin();it!=col.end();++it)
		{
			typename SPARSEBLOCKHESSIAN::TEntry & entry = it->second;

			// Compute: Hij = \Sum_k  J_{ki}^t * \Lambda_k *  J_{kj}

			typename SPARSEBLOCKHESSIAN::matrix_t Hij;
			Hij.setZero();
			const size_t nJacobs = entry.sym.lst_jacob_blocks.size();
			for (size_t k=0;k<nJacobs;k++)
			{
				const typename SPARSEBLOCKHESSIAN::symbolic_t::THessianSymbolicInfoEntry & sym_k = entry.sym.lst_jacob_blocks[k];

				if (*sym_k.J1_valid && *sym_k.J2_valid)
				{
					// Accumulate Hessian sub-blocks:
					RBA_OPTIONS::obs_noise_matrix_t::template accum_JtJ(Hij, *sym_k.J1, *sym_k.J2, sym_k.obs_idx, this->parameters.obs_noise );
				}
				else nInvalid++;
			}

			// Do scaling (if applicable):
			RBA_OPTIONS::obs_noise_matrix_t::template scale_H(Hij, this->parameters.obs_noise );

			entry.num = Hij;
		}
	}
	return nInvalid;
} // end of sparse_hessian_update_numeric

} } // end NS
