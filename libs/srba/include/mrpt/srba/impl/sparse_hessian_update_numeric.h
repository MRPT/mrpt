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

/** Rebuild the Hessian symbolic information from the internal pointers to blocks of Jacobians.
	*  Only the upper triangle is filled-in (all what is needed for Cholesky) for square Hessians, in whole for rectangular ones (it depends on the symbolic decomposition, done elsewhere).
	* \tparam SPARSEBLOCKHESSIAN can be: TSparseBlocksHessian_6x6, TSparseBlocksHessian_3x3 or TSparseBlocksHessian_6x3
	* \return The number of Jacobian multiplications skipped due to its observation being marked as "invalid"
	*/
template <class KF2KF_POSE_TYPE,class LM_TYPE,class OBS_TYPE,class RBA_OPTIONS>
template <class SPARSEBLOCKHESSIAN>
size_t RbaEngine<KF2KF_POSE_TYPE,LM_TYPE,OBS_TYPE,RBA_OPTIONS>::sparse_hessian_update_numeric( SPARSEBLOCKHESSIAN & H ) const
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
			//const size_t nJacobs = entry.sym.lst_jacob_blocks.size();
			//for (size_t k=0;k<nJacobs;k++)
			const typename SPARSEBLOCKHESSIAN::symbolic_t::list_jacob_blocks_t::const_iterator itJ_end = entry.sym.lst_jacob_blocks.end();
			for (typename SPARSEBLOCKHESSIAN::symbolic_t::list_jacob_blocks_t::const_iterator itJ = entry.sym.lst_jacob_blocks.begin(); itJ!=itJ_end; ++itJ)
			{
				const typename SPARSEBLOCKHESSIAN::symbolic_t::THessianSymbolicInfoEntry & sym_k = *itJ;

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
