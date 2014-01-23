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


/** Rebuild the Hessian symbolic information from the given Jacobians
  * Example of the expected template types:
  *  - HESS_Apf:  MatrixBlockSparseCols<double,6,3,THessianSymbolicInfo<double,2,6,3>, false >
  *  - JACOB_COLUMN_dh_dAp: TSparseBlocksJacobians_dh_dAp::col_t = MatrixBlockSparseCols<double,2,6,TJacobianSymbolicInfo_dh_dAp, false>::col_t
  */
template <class KF2KF_POSE_TYPE,class LM_TYPE,class OBS_TYPE,class RBA_OPTIONS>
template <class HESS_Ap, class HESS_f,class HESS_Apf, class JACOB_COLUMN_dh_dAp,class JACOB_COLUMN_dh_df>
void RbaEngine<KF2KF_POSE_TYPE,LM_TYPE,OBS_TYPE,RBA_OPTIONS>::sparse_hessian_build_symbolic(
	HESS_Ap & HAp,
	HESS_f & Hf,
	HESS_Apf & HApf,
	const std::vector<JACOB_COLUMN_dh_dAp*> & dh_dAp,
	const std::vector<JACOB_COLUMN_dh_df*>  & dh_df)
{
	typedef typename HESS_Ap::symbolic_t::THessianSymbolicInfoEntry  hess_Ap_sym_entry_t;   // These are concrete instances of THessianSymbolicInfo<...>::THessianSymbolicInfoEntry
	typedef typename HESS_f::symbolic_t::THessianSymbolicInfoEntry   hess_f_sym_entry_t;
	typedef typename HESS_Apf::symbolic_t::THessianSymbolicInfoEntry hess_Apf_sym_entry_t;

	const size_t nUnknowns_k2k = dh_dAp.size();
	const size_t nUnknowns_k2f = dh_df.size();

	// --------------------------------------------------------------------
	//  (1) HAp = J_Ap^t * J_Ap
	// --------------------------------------------------------------------
	// Only upper-triangular half of U (what is used by Cholesky):
	HAp.setColCount (nUnknowns_k2k);
	for (size_t i=0;i<nUnknowns_k2k;i++)
	{
		const JACOB_COLUMN_dh_dAp & col_i = *dh_dAp[i];
		ASSERT_(!col_i.empty()) // I guess this shouldn't happen...

		// j=i
		// -----
		{
			typename HESS_Ap::symbolic_t & Hii_sym = HAp.getCol(i)[i].sym;

			for (typename JACOB_COLUMN_dh_dAp::const_iterator it=col_i.begin();it!=col_i.end();++it)
			{
				const size_t obs_idx = it->first;

				Hii_sym.lst_jacob_blocks.push_back(
					hess_Ap_sym_entry_t(
						&it->second.num, &it->second.num, // J1, J2,
						it->second.sym.is_valid,it->second.sym.is_valid, // J1_valid, J2_valid,
						obs_idx
						) );
			}
		}

		// j=[i+1, nUnknowns-1]
		// -------------------
		for (size_t j=i+1;j<nUnknowns_k2k;j++)
		{
			const JACOB_COLUMN_dh_dAp & col_j = *dh_dAp[j];
			ASSERTMSG_(!col_j.empty(),mrpt::format("col_j,i=%u,j=%u empty (nUnknowns_k2k=%u)!",static_cast<unsigned int>(i),static_cast<unsigned int>(j),static_cast<unsigned int>(nUnknowns_k2k))) // I guess this shouldn't happen...

			// code below based on "std::set_intersection"
			typename JACOB_COLUMN_dh_dAp::const_iterator it_i = col_i.begin();
			typename JACOB_COLUMN_dh_dAp::const_iterator it_j = col_j.begin();
			const typename JACOB_COLUMN_dh_dAp::const_iterator it_i_end = col_i.end();
			const typename JACOB_COLUMN_dh_dAp::const_iterator it_j_end = col_j.end();

			// Don't create the entry in the sparse Hessian until we're sure it's nonzero:
			typename HESS_Ap::symbolic_t Hij_sym;

			while (it_i!=it_i_end && it_j!=it_j_end)
			{
				if ( it_i->first < it_j->first ) ++it_i;
				else if ( it_j->first < it_i->first ) ++it_j;
				else
				{
					// match between: it_i->first == it_j->first
					const size_t obs_idx = it_i->first;

					Hij_sym.lst_jacob_blocks.push_back( //std::make_pair(&it_i->second.num, &it_j->second.num) );
						hess_Ap_sym_entry_t(
							&it_i->second.num, &it_j->second.num, // J1, J2,
							it_i->second.sym.is_valid,it_j->second.sym.is_valid, // J1_valid, J2_valid,
							obs_idx
							) );

					// Move:
					++it_i; ++it_j;
				}
			}

			// If it was nonzero, move (swap) to its actual place, created with the [] operator in the std::map<>:
			if (!Hij_sym.lst_jacob_blocks.empty())
				Hij_sym.lst_jacob_blocks.swap( HAp.getCol(j)[i].sym.lst_jacob_blocks );  // Caution!! Look: at (j,i), in that order, OK?

		} // end for j
	} // end for i

	// --------------------------------------------------------------------
	//  (2) Hf = J_f^t * J_f
	// --------------------------------------------------------------------
	// Only upper-triangular half of U (what is used by Cholesky):
	Hf.setColCount (nUnknowns_k2f);
	for (size_t i=0;i<nUnknowns_k2f;i++)
	{
		const JACOB_COLUMN_dh_df & col_i = *dh_df[i];
		ASSERT_(!col_i.empty()) // I guess this shouldn't happen...

		// j=i
		// -----
		{
			typename HESS_f::symbolic_t & Hii_sym = Hf.getCol(i)[i].sym;

			for (typename JACOB_COLUMN_dh_df::const_iterator it=col_i.begin();it!=col_i.end();++it)
			{
				const size_t obs_idx = it->first;

				Hii_sym.lst_jacob_blocks.push_back( //std::make_pair(&it->second.num, &it->second.num) );
					hess_f_sym_entry_t(
						&it->second.num, &it->second.num, // J1, J2,
						it->second.sym.is_valid,it->second.sym.is_valid, // J1_valid, J2_valid,
						obs_idx
						) );
			}
		}

		// j=[i+1, nUnknowns-1]
		// -------------------
		for (size_t j=i+1;j<nUnknowns_k2f;j++)
		{
			const JACOB_COLUMN_dh_df & col_j = *dh_df[j];
			ASSERT_(!col_j.empty()) // I guess this shouldn't happen...

			// code below based on "std::set_intersection"
			typename JACOB_COLUMN_dh_df::const_iterator it_i = col_i.begin();
			typename JACOB_COLUMN_dh_df::const_iterator it_j = col_j.begin();
			const typename JACOB_COLUMN_dh_df::const_iterator it_i_end = col_i.end();
			const typename JACOB_COLUMN_dh_df::const_iterator it_j_end = col_j.end();

			// Don't create the entry in the sparse Hessian until we're sure it's nonzero:
			typename HESS_f::symbolic_t Hij_sym;

			while (it_i!=it_i_end && it_j!=it_j_end)
			{
				if ( it_i->first < it_j->first ) ++it_i;
				else if ( it_j->first < it_i->first ) ++it_j;
				else
				{
					// match between: it_i->first == it_j->first
					const size_t obs_idx = it_i->first;

					Hij_sym.lst_jacob_blocks.push_back( //std::make_pair(&it_i->second.num, &it_j->second.num) );
						hess_f_sym_entry_t(
							&it_i->second.num, &it_j->second.num, // J1, J2,
							it_i->second.sym.is_valid,it_j->second.sym.is_valid, // J1_valid, J2_valid,
							obs_idx
							) );

					// Move:
					++it_i; ++it_j;
				}
			}

			// If it was nonzero, move (swap) to its actual place, created with the [] operator in the std::map<>:
			if (!Hij_sym.lst_jacob_blocks.empty())
				Hij_sym.lst_jacob_blocks.swap( Hf.getCol(j)[i].sym.lst_jacob_blocks );  // Caution!! Look: at (j,i), in that order, OK?

		} // end for j
	} // end for i

	// --------------------------------------------------------------------
	//  (3) HApf = J_Ap^t * J_f
	// --------------------------------------------------------------------
	// The entire rectangular matrix (it's NOT symetric!)

	// *NOTE* HApf will be stored indices by rows instead of columns!!

	//HApf.setColCount(nUnknowns_k2f);
	HApf.setColCount(nUnknowns_k2k);  // # of ROWS
	for (size_t i=0;i<nUnknowns_k2k;i++)  // i:row = [0,nUnknowns_k2k-1]
	{
		const JACOB_COLUMN_dh_dAp & col_i = *dh_dAp[i];
		ASSERT_(!col_i.empty()) // I guess this shouldn't happen...

		// j:col = [0,nUnknowns_k2f-1]
		for (size_t j=0;j<nUnknowns_k2f;j++)
		{
			const JACOB_COLUMN_dh_df & col_j = *dh_df[j];
			ASSERT_(!col_j.empty()) // I guess this shouldn't happen...

			// code below based on "std::set_intersection"
			typename JACOB_COLUMN_dh_dAp::const_iterator it_i = col_i.begin();
			typename JACOB_COLUMN_dh_df::const_iterator it_j = col_j.begin();
			const typename JACOB_COLUMN_dh_dAp::const_iterator it_i_end = col_i.end();
			const typename JACOB_COLUMN_dh_df::const_iterator it_j_end = col_j.end();

			// Don't create the entry in the sparse Hessian until we're sure it's nonzero:
			typename HESS_Apf::symbolic_t Hij_sym;

			while (it_i!=it_i_end && it_j!=it_j_end)
			{
				if ( it_i->first < it_j->first ) ++it_i;
				else if ( it_j->first < it_i->first ) ++it_j;
				else
				{
					// match between: it_i->first == it_j->first
					const size_t obs_idx = it_i->first;

					Hij_sym.lst_jacob_blocks.push_back(
						hess_Apf_sym_entry_t(
							&it_i->second.num, &it_j->second.num, // J1, J2,
							it_i->second.sym.is_valid,it_j->second.sym.is_valid, // J1_valid, J2_valid,
							obs_idx
							) );

					// Move:
					++it_i; ++it_j;
				}
			}

			// If it was nonzero, move (swap) to its actual place, created with the [] operator in the std::map<>:
			if (!Hij_sym.lst_jacob_blocks.empty())
				Hij_sym.lst_jacob_blocks.swap( HApf.getCol(i)[j].sym.lst_jacob_blocks ); // (i,j) because it's stored indices by rows instead of columns!

		} // end for j
	} // end for i

}

} } // end NS
