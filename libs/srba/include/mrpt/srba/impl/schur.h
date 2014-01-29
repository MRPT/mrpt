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

	/** A generic symbolic and numeric Schur-complement handler for builing reduced systems of equations.
	  */
	template <class HESS_Ap, class HESS_f, class HESS_Apf>
	class SchurComplement
	{
	public:

		/** Constructor: builds the symbolic representations
		  *  Note: HApf must be in row-compressed form; HAp & Hf in column-compressed form.
		  * \param[in] _minus_grad_f Can be NULL if there're no observations of landmarks with unknown positions (may still be of LMs with known ones).
		  */
		SchurComplement(HESS_Ap  &_HAp, HESS_f & _Hf, HESS_Apf & _HApf, double * _minus_grad_Ap, double * _minus_grad_f)
		: HAp(_HAp), Hf(_Hf), HApf(_HApf),
		  minus_grad_Ap(_minus_grad_Ap),
		  minus_grad_f(_minus_grad_f),
		  // Problem dims:
		  nUnknowns_Ap( HAp.getColCount() ),
		  nUnknowns_f( Hf.getColCount() ),
		  nHf_invertible_blocks(0)
		{
			if (!nUnknowns_f || !nUnknowns_Ap) return;

			// 0) Make copy of the original numerical values of HAp:
			// -----------------------------------------------------------------
			HAp_original.copyNumericalValuesFrom( HAp );

			// 1) List of all diagonal blocks in Hf which have to be inverted
			// -----------------------------------------------------------------
			m_Hf_blocks_info.resize(nUnknowns_f);
			for (size_t i=0;i<nUnknowns_f;i++)
			{
				const typename HESS_f::col_t & col_i = Hf.getCol(i);
				ASSERT_(!col_i.empty() && col_i.rbegin()->first==i)

				m_Hf_blocks_info[i].sym_Hf_diag_blocks = &col_i.rbegin()->second.num;
			}

			// 2) Build instructions to reduce H_Ap and grad_Ap
			// ----------------------------------------------------
			m_sym_HAp_reduce.clear();
			m_sym_GradAp_reduce.resize(nUnknowns_Ap);

			for (size_t i=0;i<nUnknowns_Ap;i++)
			{	// Only upper-half triangle:
				typename HESS_Ap::col_t & HAp_col_i = HAp.getCol(i);

				// Go thru all the "j" (row) indices in j \in [0,i]
				// even if there's not an entry in the column map "HAp_col_i" (yet).
				// If the block (i,j) has any feature in common, then we'll add
				// a block HAp_{i,j} right here and add the corresponding instructions for building the numeric Schur.
				for (size_t j=0;j<=i;j++)
				{
					// We are at the HAp block (i,j):
					// Make a list of all the:
					//  \Sum H_{pi,lk} * H_{lk}^{-1} *  H_{pj,lk}^t
					//
					// This amounts to looking for the "intersecting set" of the two rows "i" & "j" in HApf:
					THApSymbolicEntry  sym_ij;

					// (i==j) -> take advantage of repeated terms and build instructions for the gradient:
					TGradApSymbolicEntry *grad_entries = (i==j) ? &m_sym_GradAp_reduce[i] : NULL;


					// get Rows i & j from HAp_f:
					typename HESS_Apf::col_t  & row_i = HApf.getCol(i);
					typename HESS_Apf::col_t  & row_j = HApf.getCol(j);

					// code below based on "std::set_intersection"
					typename HESS_Apf::col_t::const_iterator it_i = row_i.begin();
					typename HESS_Apf::col_t::const_iterator it_j = row_j.begin();
					const typename HESS_Apf::col_t::const_iterator it_i_end = row_i.end();
					const typename HESS_Apf::col_t::const_iterator it_j_end = row_j.end();

					while (it_i!=it_i_end && it_j!=it_j_end)
					{
						if ( it_i->first < it_j->first ) ++it_i;
						else if ( it_j->first < it_i->first ) ++it_j;
						else
						{
							// match between: it_i->first == it_j->first

							// Add a new triplet:
							//  * const typename HESS_Apf::matrix_t * Hpi_lk;
							//  * const typename HESS_f::matrix_t   * inv_Hf_lk;
							//  * const typename HESS_Apf::matrix_t * Hpj_lk;

							const size_t idx_feat = it_j->first;
							ASSERT_(idx_feat<nUnknowns_f)

							// Gradient (only if we're at i==j)
							typename HESS_Apf::matrix_t *out_temporary_result = NULL;
							if (grad_entries)
							{
								grad_entries->lst_terms_to_subtract.resize( grad_entries->lst_terms_to_subtract.size()+1 );
								typename TGradApSymbolicEntry::TEntry  &ent = *grad_entries->lst_terms_to_subtract.rbegin();
								ent.feat_idx = idx_feat;
								out_temporary_result = &ent.Hpi_lk_times_inv_Hf_lk;
							}

							// Hessian:
#if 0
							std::cout << "SymSchur.HAp("<<i<<","<<j<< "): HApf_" << it_i->first << " * HApf_" << it_j->first  << std::endl;
#endif
							sym_ij.lst_terms_to_add.push_back( typename THApSymbolicEntry::TEntry(
								&it_j->second.num, //&it_i->second.num,
								&m_Hf_blocks_info[idx_feat],
								&it_i->second.num, //&it_j->second.num,
								out_temporary_result ) );

							// Move:
							++it_i; ++it_j;
						}
					} // end while (find intersect)

					// Only append if not empty:
					if (!sym_ij.lst_terms_to_add.empty())
					{
						// There are common observations between Api & Apj:
						// 1) If there was already an HAp_ij block, take a reference to it.
						// 2) Otherwise, insert it now.
						typename HESS_Ap::col_t::iterator itExistingRowEntry;
						if ( ((i==j) || (i!=j && HAp_col_i.size()!=1)) // If size==1 don't even waste time: it's a diagonal block.
							&&
							 (itExistingRowEntry=HAp_col_i.find(j))!= HAp_col_i.end())
						{
							// Add reference to target matrix for numeric Schur:
							sym_ij.HAp_ij = & itExistingRowEntry ->second.num;
						}
						else
						{
							ASSERT_(i!=j)  // We should only reach here for off-diagonal blocks!
							// Create & add reference to target matrix for numeric Schur:
							sym_ij.HAp_ij = & HAp_col_i[j].num;
							// Clear initial contents of the Hessian to all zeros:
							sym_ij.HAp_ij->setZero();
						}

						// Fast move into the deque:
						m_sym_HAp_reduce.resize( m_sym_HAp_reduce.size()+1 );
						sym_ij.swap( *m_sym_HAp_reduce.rbegin() );
					}

				} // end for j (row in HAp)
			} // end for i (col in HAp)

		} // end of ctor.

		/** Must be called after the numerical values of the Hessian HAp change, typically after an optimization update
		  * which led to re-evaluation of the Jacobians around a new linearization point. This method latches the current
		  * value of HAp for reseting it again if we later need to recompute the reduced Schur system for different values of lambda.
		  */
		void realize_HAp_changed()
		{
			HAp_original.copyNumericalValuesFrom( HAp );
		}

		/** After calling numeric_build_reduced_system() one can get the stats on how many features are actually estimable */
		size_t getNumFeatures() const { return nUnknowns_f; }
		size_t getNumFeaturesFullRank() const { return nHf_invertible_blocks; }


		/** Replace the HAp matrix with its Schur reduced version.
		  * The lambda value is used to sum it to the diagonal of Hf (features), but it's NOT added
		  *  to the output HAp. This is intentional, so the caller can add different lamdba values
		  *  as needed in different trials if an ill conditioned matrix is found.
		  */
		void numeric_build_reduced_system(const double lambda)
		{
			if (!nUnknowns_f || !nUnknowns_Ap) return;

			// 0) Restore the original state of the Hessian (since all changes are defined as sums / substractions on it)
			//     and this operation can be invoked several times with the same original HAp but diferent lambda values
			//     (which forces a total recomputation due to the inv(Hf+\lambda*I) terms).
			// --------------------------------------------------------------------------------
			HAp.copyNumericalValuesFrom( HAp_original );

			// 1) Invert diagonal blocks in Hf:
			// ---------------------------------
			nHf_invertible_blocks=0;
			for (size_t i=0;i<nUnknowns_f;i++)
			{
				// LU decomposition is rank-revealing (not like LLt)
				typename HESS_f::matrix_t Hfi = *m_Hf_blocks_info[i].sym_Hf_diag_blocks;
				for (int k=0;k<Hfi.cols();k++)
					Hfi.coeffRef(k,k)+=lambda;

				const Eigen::FullPivLU<typename HESS_f::matrix_t> lu( Hfi );

				// Badly conditioned matrix?
				if (true== (m_Hf_blocks_info[i].num_Hf_diag_blocks_invertible = lu.isInvertible() ))
				{
					nHf_invertible_blocks++;
					m_Hf_blocks_info[i].num_Hf_diag_blocks_inverses = lu.inverse();
				}
			}

			// 2) H_Ap of the reduced system:
			// ---------------------------------
			typename HESS_Apf::matrix_t aux_Hpi_lk_times_inv_Hf_lk;
			for (typename std::deque<THApSymbolicEntry>::const_iterator it=m_sym_HAp_reduce.begin();it!=m_sym_HAp_reduce.end();++it)
			{
				const THApSymbolicEntry &sym_entry = *it;

				typename HESS_Ap::matrix_t & HAp_ij = *sym_entry.HAp_ij;

				const size_t N = sym_entry.lst_terms_to_add.size();

				//std::cout << "before:\n" << HAp_ij << std::endl;
				for (size_t i=0;i<N;i++)
				{
					const typename THApSymbolicEntry::TEntry &entry = sym_entry.lst_terms_to_add[i];

					if (entry.inv_Hf_lk->num_Hf_diag_blocks_invertible)
					{
						//cout << "Hfinv:\n" << entry.inv_Hf_lk->num_Hf_diag_blocks_inverses  << endl;
						// \bar{Hp} -=  Hpi_lk * inv(Hf_lk) * Hpj_lk^t

						// Store this term for reuse with the gradient update, or use temporary local storage:
						typename HESS_Apf::matrix_t * Hpi_lk_times_inv_Hf_lk =
							entry.out_Hpi_lk_times_inv_Hf_lk!=NULL
							?
							entry.out_Hpi_lk_times_inv_Hf_lk : &aux_Hpi_lk_times_inv_Hf_lk;

						Hpi_lk_times_inv_Hf_lk->noalias() = (*entry.Hpi_lk) * entry.inv_Hf_lk->num_Hf_diag_blocks_inverses;

						//std::cout << "product #" << i << ":\nHpi_lk:\n" << (*entry.Hpi_lk) << "\nHfi^-1:\n" << entry.inv_Hf_lk->num_Hf_diag_blocks_inverses << "\nHpj_lk^t:\n" << entry.Hpj_lk->transpose() << "\n\n";
						HAp_ij.noalias() -= (*Hpi_lk_times_inv_Hf_lk) * (*entry.Hpj_lk).transpose();
					}
				}
				//std::cout << "after:\n" << HAp_ij<< std::endl;
			}

			// 3) g_Ap of the reduced system:
			// ---------------------------------
			double * modified_grad_Ap = minus_grad_Ap;
			for (size_t i=0;i<nUnknowns_Ap;i++)
			{
				vector_Ap_t grad_Ap = vector_Ap_t(modified_grad_Ap); // A map which wraps the pointer
				for (typename TGradApSymbolicEntry::lst_terms_t::const_iterator it=m_sym_GradAp_reduce[i].lst_terms_to_subtract.begin();it!=m_sym_GradAp_reduce[i].lst_terms_to_subtract.end();++it)
				{
					if (m_Hf_blocks_info[it->feat_idx].num_Hf_diag_blocks_invertible)
					{
						double *grad_df = this->minus_grad_f + it->feat_idx * HESS_f::matrix_t::RowsAtCompileTime;
						// g -= \Sum Hpi_lk * inv(Hf_lk) * grad_lk
						//
						grad_Ap.noalias() -= it->Hpi_lk_times_inv_Hf_lk * vector_f_t(grad_df);
					}
				}

				// Advance to next part of gradient:
				modified_grad_Ap+= HESS_Ap::matrix_t::RowsAtCompileTime;
			}


		} // end of numeric_build_reduced_system


		void numeric_solve_for_features(
			double *in_deltas_Ap,
			double *out_deltas_feats
			)
		{
			// 1/2: Go thru all the Hessian blocks of HApl and subtracts the corresponding term to
			//       the grad_feats:
			for (size_t idx_Ap=0;idx_Ap<nUnknowns_Ap;idx_Ap++)
			{
				// get Row i from HAp_f:
				typename HESS_Apf::col_t  & row_i = HApf.getCol(idx_Ap);

				const vector_Ap_t delta_idx_Ap = vector_Ap_t(in_deltas_Ap + idx_Ap * HESS_Ap::matrix_t::RowsAtCompileTime );

				for (typename HESS_Apf::col_t::const_iterator itCol=row_i.begin();itCol!=row_i.end();++itCol)
				{
					const size_t idx_feat = itCol->first;
					if (!was_ith_feature_invertible(idx_feat))
						continue;

					double *grad_df = this->minus_grad_f + idx_feat * HESS_f::matrix_t::RowsAtCompileTime;

					// g_reduced = -g_l - \Sum H^t_pi_lk * delta_Ap_i
					vector_f_t(grad_df).noalias() -= itCol->second.num.transpose() * delta_idx_Ap;
				}
			}

			// 2/2: Solve each individual feature increment:
			for (size_t idx_feat=0;idx_feat<nUnknowns_f;idx_feat++)
			{
				if (!was_ith_feature_invertible(idx_feat))
					continue;

				vector_f_t delta_feat = vector_f_t(  out_deltas_feats + idx_feat * HESS_f::matrix_t::RowsAtCompileTime );
				vector_f_t grad_df    = vector_f_t(this->minus_grad_f + idx_feat * HESS_f::matrix_t::RowsAtCompileTime );

				//std::cout  << grad_df.transpose() << std::endl << m_Hf_blocks_info[idx_feat].num_Hf_diag_blocks_inverses << std::endl << std::endl;

				delta_feat = (m_Hf_blocks_info[idx_feat].num_Hf_diag_blocks_inverses * grad_df);
			}

		} // end of numeric_solve_for_features


		bool was_ith_feature_invertible(const size_t i) const { return m_Hf_blocks_info[i].num_Hf_diag_blocks_invertible; }

	private:
		// ----------- Input data ------------------
		HESS_Ap    HAp_original;  // (Copy of the numerical values of HAp)
		HESS_Ap  & HAp;  // Column compressed
		HESS_f   & Hf;   // Column compressed
		HESS_Apf & HApf; // *ROW* compressed
		double   * minus_grad_Ap;
		double   * minus_grad_f;   // Should be changed to "const" if used a Eigen::"ConstMap"...

		const size_t nUnknowns_Ap;
		const size_t nUnknowns_f;
		size_t nHf_invertible_blocks; //!< for stats, the number of Hf diagonal blocks which are not rank-deficient.
		// -----------------------------------------
		typedef typename Eigen::Map<Eigen::Matrix<double,HESS_Ap::matrix_t::RowsAtCompileTime,1> > vector_Ap_t;
		typedef typename Eigen::Map<Eigen::Matrix<double,HESS_f::matrix_t::RowsAtCompileTime,1> > vector_f_t;


		struct TInfoPerHfBlock
		{
			const typename HESS_f::matrix_t * sym_Hf_diag_blocks;
			typename HESS_f::matrix_t         num_Hf_diag_blocks_inverses;
			bool                              num_Hf_diag_blocks_invertible; //!< Whether \a num_Hf_diag_blocks_inverses could be generated

			TInfoPerHfBlock() : sym_Hf_diag_blocks(NULL), num_Hf_diag_blocks_invertible(false) { }

			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		};

		typedef typename mrpt::aligned_containers<TInfoPerHfBlock>::vector_t TInfoPerHfBlock_vector_t;

		TInfoPerHfBlock_vector_t  m_Hf_blocks_info;

		/** Info for each block in HAp */
		struct THApSymbolicEntry
		{
			struct TEntry
			{
				TEntry(
					const typename HESS_Apf::matrix_t * _Hpi_lk,
					const TInfoPerHfBlock             * _inv_Hf_lk,
					const typename HESS_Apf::matrix_t * _Hpj_lk,
					typename HESS_Apf::matrix_t       * _out_Hpi_lk_times_inv_Hf_lk
					)
					:
						Hpi_lk(_Hpi_lk),
						inv_Hf_lk(_inv_Hf_lk),
						Hpj_lk(_Hpj_lk),
						out_Hpi_lk_times_inv_Hf_lk(_out_Hpi_lk_times_inv_Hf_lk)
				{
				}

				const typename HESS_Apf::matrix_t * Hpi_lk;
				const TInfoPerHfBlock             * inv_Hf_lk;
				const typename HESS_Apf::matrix_t * Hpj_lk;
				typename HESS_Apf::matrix_t       * out_Hpi_lk_times_inv_Hf_lk;  //!< If NULL=use local storage.
			};

			typename HESS_Ap::matrix_t * HAp_ij;
			std::deque<TEntry>          lst_terms_to_add;

			void swap(THApSymbolicEntry &o)
			{
				std::swap(HAp_ij,o.HAp_ij);
				lst_terms_to_add.swap(o.lst_terms_to_add);
			}
		};

		std::deque<THApSymbolicEntry>  m_sym_HAp_reduce;  //!< All the required operations needed to update HAp into its reduced system.


		struct TGradApSymbolicEntry
		{
			struct TEntry
			{
				size_t feat_idx;
				// Used as a temporary container for the product, but also because this term reappears in the gradient update:
				typename HESS_Apf::matrix_t         Hpi_lk_times_inv_Hf_lk;

				EIGEN_MAKE_ALIGNED_OPERATOR_NEW
			};

// BUG ALERT in Eigen/Deque/MSVC 2008 32bit: http://eigen.tuxfamily.org/bz/show_bug.cgi?id=83
// This workaround will lose performance but at least allow compiling. Remove this if someday a solution
// to the bug above is found:
#if defined(_MSC_VER) && (_MSC_VER < 1600)  // if we use MSVC older than 2010:
			typedef typename mrpt::aligned_containers<TEntry>::list_t lst_terms_t;  // slower than deque, but works...
#else
			// (Warning: Don't replace the STL container with "vector" or any other that invalidate
			// pointers, which is assumed in the implementation. That's why I use "std::deque")
			typedef typename mrpt::aligned_containers<TEntry>::deque_t lst_terms_t;
#endif
			// The list itself:
			lst_terms_t  lst_terms_to_subtract;
		};
		std::vector<TGradApSymbolicEntry> m_sym_GradAp_reduce;  //!< All the required operations needed to update Gradient of Ap into its reduced system.

	};  // end of class SchurComplement

} } // end of namespaces

