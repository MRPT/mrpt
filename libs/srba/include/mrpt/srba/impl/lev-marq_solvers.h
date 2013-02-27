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

#include <mrpt/math/CSparseMatrix.h>
#include <memory> // for auto_ptr, unique_ptr

namespace mrpt { namespace srba {

using namespace mrpt;
using namespace mrpt::math;
using namespace std;

namespace internal
{
#if MRPT_HAS_CXX11
	typedef std::unique_ptr<CSparseMatrix::CholeskyDecomp>  SparseCholeskyDecompPtr;
#else
	typedef std::auto_ptr<CSparseMatrix::CholeskyDecomp>    SparseCholeskyDecompPtr;
#endif

	// ------------------------------------------------------------------------------------------
	/** SOLVER: Lev-Marq without Schur, with Sparse Cholesky (CSparse library)  */
	template <class RBA_ENGINE>
	struct solver_engine<false /*Schur*/,false /*dense Chol*/,RBA_ENGINE>
	{
		typedef typename RBA_ENGINE::hessian_traits_t hessian_traits_t;

		static const size_t POSE_DIMS = RBA_ENGINE::kf2kf_pose_type::REL_POSE_DIMS;
		static const size_t LM_DIMS   = RBA_ENGINE::lm_type::LM_DIMS;

		const int m_verbose_level;
		mrpt::utils::CTimeLogger &m_profiler;
		vector_double  delta_eps; //!< The result of solving Ax=b will be stored here
		typename hessian_traits_t::TSparseBlocksHessian_Ap  &HAp;
		typename hessian_traits_t::TSparseBlocksHessian_f   &Hf;
		typename hessian_traits_t::TSparseBlocksHessian_Apf &HApf;
		vector_double  &minus_grad;
		const size_t nUnknowns_k2k, nUnknowns_k2f, nUnknowns_scalars,idx_start_f;

		CSparseMatrix* sS; //!< Sparse Hessian
		bool           sS_is_valid; //!< Whether the Hessian was filled in, in sS
		/** Cholesky object, as a pointer to reuse it between iterations */
		SparseCholeskyDecompPtr ptrCh;

		/** Constructor */
		solver_engine(
			const int verbose_level,
			mrpt::utils::CTimeLogger & profiler,
			typename hessian_traits_t::TSparseBlocksHessian_Ap  &HAp_,
			typename hessian_traits_t::TSparseBlocksHessian_f   &Hf_,
			typename hessian_traits_t::TSparseBlocksHessian_Apf &HApf_,
			vector_double  &minus_grad_,
			const size_t nUnknowns_k2k_,
			const size_t nUnknowns_k2f_) :
				m_verbose_level(verbose_level),
				m_profiler(profiler),
				HAp(HAp_), Hf(Hf_), HApf(HApf_),
				minus_grad(minus_grad_),
				nUnknowns_k2k(nUnknowns_k2k_),
				nUnknowns_k2f(nUnknowns_k2f_),
				nUnknowns_scalars( POSE_DIMS*nUnknowns_k2k + LM_DIMS*nUnknowns_k2f ),
				idx_start_f(POSE_DIMS*nUnknowns_k2k),
				sS(NULL), sS_is_valid(false)
		{
		}

		~solver_engine()
		{
			if (sS) { delete sS; sS=NULL; }
		}

		// ----------------------------------------------------------------------
		// Solve the entire H Ax = -g system with H as a single sparse Hessian.
		// Return: true on success. false to retry with a different lambda (Lev-Marq algorithm is assumed)
		// ----------------------------------------------------------------------
		bool solve(const double lambda)
		{
			DETAILED_PROFILING_ENTER("opt.SparseTripletFill")

			if (!sS) sS = new CSparseMatrix(nUnknowns_scalars,nUnknowns_scalars);
			else     sS->clear(nUnknowns_scalars,nUnknowns_scalars);
			sS_is_valid=false;

			// 1/3: Hp --------------------------------------
			for (size_t i=0;i<nUnknowns_k2k;i++)
			{	// Only upper-half triangle:
				const typename hessian_traits_t::TSparseBlocksHessian_Ap::col_t & col_i = HAp.getCol(i);

				for (typename hessian_traits_t::TSparseBlocksHessian_Ap::col_t::const_iterator itRowEntry = col_i.begin();itRowEntry != col_i.end(); ++itRowEntry )
				{
					if (itRowEntry->first==i)
					{
						// block Diagonal: Add lambda*I to these ones
						typename hessian_traits_t::TSparseBlocksHessian_Ap::matrix_t sSii = itRowEntry->second.num;
						for (size_t k=0;k<POSE_DIMS;k++)
							sSii.coeffRef(k,k)+=lambda;
						sS->insert_submatrix(POSE_DIMS*i,POSE_DIMS*i, sSii );
					}
					else
					{
						sS->insert_submatrix(
							POSE_DIMS*itRowEntry->first,  // row index
							POSE_DIMS*i,                  // col index
							itRowEntry->second.num );
					}
				}
			}

			// 2/3: HApf --------------------------------------
			for (size_t i=0;i<nUnknowns_k2k;i++)
			{
				const typename hessian_traits_t::TSparseBlocksHessian_Apf::col_t & row_i = HApf.getCol(i);

				for (typename hessian_traits_t::TSparseBlocksHessian_Apf::col_t::const_iterator itColEntry = row_i.begin();itColEntry != row_i.end(); ++itColEntry )
				{
					sS->insert_submatrix(
						POSE_DIMS*i,  // row index
						idx_start_f+LM_DIMS*itColEntry->first,  // col index
						itColEntry->second.num );
				}
			}

			// 3/3: Hf --------------------------------------
			for (size_t i=0;i<nUnknowns_k2f;i++)
			{	// Only upper-half triangle:
				const typename hessian_traits_t::TSparseBlocksHessian_f::col_t & col_i = Hf.getCol(i);

				for (typename hessian_traits_t::TSparseBlocksHessian_f::col_t::const_iterator itRowEntry = col_i.begin();itRowEntry != col_i.end(); ++itRowEntry )
				{
					if (itRowEntry->first==i)
					{
						// block Diagonal: Add lambda*I to these ones
						typename hessian_traits_t::TSparseBlocksHessian_f::matrix_t sSii = itRowEntry->second.num;
						for (size_t k=0;k<LM_DIMS;k++)
							sSii.coeffRef(k,k)+=lambda;
						sS->insert_submatrix(idx_start_f+LM_DIMS*i,idx_start_f+LM_DIMS*i, sSii );
					}
					else
					{
						sS->insert_submatrix(
							idx_start_f+LM_DIMS*itRowEntry->first,  // row index
							idx_start_f+LM_DIMS*i,                  // col index
							itRowEntry->second.num );
					}
				}
			}
			DETAILED_PROFILING_LEAVE("opt.SparseTripletFill")

			// Compress the sparse matrix:
			// --------------------------------------
			DETAILED_PROFILING_ENTER("opt.SparseTripletCompress")
			sS->compressFromTriplet();
			DETAILED_PROFILING_LEAVE("opt.SparseTripletCompress")

			// Sparse cholesky
			// --------------------------------------
			DETAILED_PROFILING_ENTER("opt.SparseChol")
			try
			{
				if (!ptrCh.get())
						ptrCh = SparseCholeskyDecompPtr(new CSparseMatrix::CholeskyDecomp(*sS) );
				else ptrCh.get()->update(*sS);

				sS_is_valid=true;

				DETAILED_PROFILING_LEAVE("opt.SparseChol")
			}
			catch (CExceptionNotDefPos &)
			{
				DETAILED_PROFILING_LEAVE("opt.SparseChol")
				return false;
			}

			// backsubtitution gives us "DeltaEps" from Cholesky and "-grad":
			//
			//    (J^tJ + lambda*I) DeltaEps = -grad
			// ----------------------------------------------------------------
			DETAILED_PROFILING_ENTER("opt.backsub")
			ptrCh->backsub(minus_grad,delta_eps);
			DETAILED_PROFILING_LEAVE("opt.backsub")

			return true; // solved OK.
		}

		void realize_relinearized()
		{
			// Nothing to do.
		}
		void realize_lambda_changed()
		{
			// Nothing to do.
		}
		bool was_ith_feature_invertible(const size_t i)
		{
			return true;
		}

		/** Here, out_info is of type mrpt::srba::options::solver_LM_schur_sparse_cholesky::extra_results_t */
		void get_extra_results(typename RBA_ENGINE::rba_options_type::solver_t::extra_results_t & out_info )
		{
			out_info.hessian_valid = sS_is_valid;
			if (sS) sS->swap(out_info.hessian);
		}
	};

	// ------------------------------------------------------------------------------------------
	/** SOLVER: Lev-Marq with Schur, with Sparse Cholesky (CSparse library) */
	template <class RBA_ENGINE>
	struct solver_engine<true /*Schur*/,false /*dense Chol*/,RBA_ENGINE>
	{
		typedef typename RBA_ENGINE::hessian_traits_t hessian_traits_t;

		static const size_t POSE_DIMS = RBA_ENGINE::kf2kf_pose_type::REL_POSE_DIMS;
		static const size_t LM_DIMS   = RBA_ENGINE::lm_type::LM_DIMS;

		const int m_verbose_level;
		mrpt::utils::CTimeLogger &m_profiler;

		const size_t   nUnknowns_k2k, nUnknowns_k2f;
		vector_double  delta_eps;
		typename hessian_traits_t::TSparseBlocksHessian_Ap  &HAp;
		typename hessian_traits_t::TSparseBlocksHessian_f   &Hf;
		typename hessian_traits_t::TSparseBlocksHessian_Apf &HApf;
		vector_double  &minus_grad;

		CSparseMatrix* sS; //!< Sparse Hessian
		bool           sS_is_valid; //!< Whether the Hessian was filled in, in sS
		/** Cholesky object, as a pointer to reuse it between iterations */
		SparseCholeskyDecompPtr  ptrCh;

		SchurComplement<
			typename hessian_traits_t::TSparseBlocksHessian_Ap,
			typename hessian_traits_t::TSparseBlocksHessian_f,
			typename hessian_traits_t::TSparseBlocksHessian_Apf
			>
			schur_compl;

		/** Constructor */
		solver_engine(
			const int verbose_level,
			mrpt::utils::CTimeLogger & profiler,
			typename hessian_traits_t::TSparseBlocksHessian_Ap  &HAp_,
			typename hessian_traits_t::TSparseBlocksHessian_f   &Hf_,
			typename hessian_traits_t::TSparseBlocksHessian_Apf &HApf_,
			vector_double  &minus_grad_,
			const size_t nUnknowns_k2k_,
			const size_t nUnknowns_k2f_) :
				m_verbose_level(verbose_level),
				m_profiler(profiler),
				nUnknowns_k2k(nUnknowns_k2k_),
				nUnknowns_k2f(nUnknowns_k2f_),
				HAp(HAp_),Hf(Hf_),HApf(HApf_),
				minus_grad(minus_grad_),
				sS(NULL), sS_is_valid(false),
				schur_compl(
					HAp_,Hf_,HApf_, // The different symbolic/numeric Hessians
					&minus_grad[0],  // minus gradient of the Ap part
					// Handle case of no unknown features:
					nUnknowns_k2f!=0 ? &minus_grad[POSE_DIMS*nUnknowns_k2k] : NULL   // minus gradient of the features part
					)
		{
		}

		~solver_engine()
		{
			if (sS) { delete sS; sS=NULL; }
		}

		// ----------------------------------------------------------------------
		// Solve the H·Ax = -g system using the Schur complement to generate a
		//   Ap-only reduced system.
		// Return: always true (success).
		// ----------------------------------------------------------------------
		bool solve(const double lambda)
		{
			// 1st: Numeric part: Update HAp hessian into the reduced system
			// Note: We have to re-evaluate the entire reduced Hessian HAp even if
			//       only lambda changed, because of the terms inv(Hf+\lambda*I).

			DETAILED_PROFILING_ENTER("opt.schur_build_reduced")
			schur_compl.numeric_build_reduced_system(lambda);
			DETAILED_PROFILING_LEAVE("opt.schur_build_reduced")

			if (schur_compl.getNumFeaturesFullRank()!=schur_compl.getNumFeatures())
				VERBOSE_LEVEL(1) << "[OPT] Schur warning: only " << schur_compl.getNumFeaturesFullRank() << " out of " << schur_compl.getNumFeatures() << " features have full-rank.\n";

			// Only the H_Ap part of the Hessian:
			if (!sS) sS = new CSparseMatrix(nUnknowns_k2k*POSE_DIMS,nUnknowns_k2k*POSE_DIMS);
			else     sS->clear(nUnknowns_k2k*POSE_DIMS,nUnknowns_k2k*POSE_DIMS);
			sS_is_valid=false;


			// Now write the updated "HAp" into its sparse matrix form:
			DETAILED_PROFILING_ENTER("opt.SparseTripletFill")
			for (size_t i=0;i<nUnknowns_k2k;i++)
			{	// Only upper-half triangle:
				const typename hessian_traits_t::TSparseBlocksHessian_Ap::col_t & col_i = HAp.getCol(i);

				for (typename hessian_traits_t::TSparseBlocksHessian_Ap::col_t::const_iterator itRowEntry = col_i.begin();itRowEntry != col_i.end(); ++itRowEntry )
				{
					if (itRowEntry->first==i)
					{
						// block Diagonal: Add lambda*I to these ones
						typename hessian_traits_t::TSparseBlocksHessian_Ap::matrix_t sSii = itRowEntry->second.num;
						for (size_t k=0;k<POSE_DIMS;k++)
							sSii.coeffRef(k,k)+=lambda;
						sS->insert_submatrix(POSE_DIMS*i,POSE_DIMS*i, sSii );
					}
					else
					{
						sS->insert_submatrix(
							POSE_DIMS*itRowEntry->first,  // row index
							POSE_DIMS*i,                  // col index
							itRowEntry->second.num );
					}
				}
			}
			DETAILED_PROFILING_LEAVE("opt.SparseTripletFill")

			// Compress the sparse matrix:
			// ----------------------------------
			DETAILED_PROFILING_ENTER("opt.SparseTripletCompress")
			sS->compressFromTriplet();
			DETAILED_PROFILING_LEAVE("opt.SparseTripletCompress")

			// Sparse cholesky:
			// ----------------------------------
			DETAILED_PROFILING_ENTER("opt.SparseChol")
			try
			{
				if (!ptrCh.get())
						ptrCh = SparseCholeskyDecompPtr(new CSparseMatrix::CholeskyDecomp(*sS) );
				else ptrCh.get()->update(*sS);

				sS_is_valid=true;

				DETAILED_PROFILING_LEAVE("opt.SparseChol")
			}
			catch (CExceptionNotDefPos &)
			{
				DETAILED_PROFILING_LEAVE("opt.SparseChol")
				// not positive definite so increase lambda and try again
				return false;
			}

			// backsubtitution gives us "DeltaEps" from Cholesky and "-grad":
			//
			//    (J^tJ + lambda*I) DeltaEps = -grad
			// ----------------------------------------------------------------
			DETAILED_PROFILING_ENTER("opt.backsub")

			delta_eps.assign(nUnknowns_k2k*POSE_DIMS + nUnknowns_k2f*LM_DIMS, 0); // Important: It's initialized to zeros!

			ptrCh->backsub(&minus_grad[0],&delta_eps[0],nUnknowns_k2k*POSE_DIMS);

			DETAILED_PROFILING_LEAVE("opt.backsub")

			// If using the Schur complement, at this point we must now solve the secondary
			//  system for features:
			// -----------------------------------------------------------------------------
			// 2nd numeric part: Solve for increments in features ----------
			DETAILED_PROFILING_ENTER("opt.schur_features")

			schur_compl.numeric_solve_for_features(
				&delta_eps[0],
				// Handle case of no unknown features:
				nUnknowns_k2f!=0 ? &delta_eps[nUnknowns_k2k*POSE_DIMS] : NULL   // minus gradient of the features part
				);

			DETAILED_PROFILING_LEAVE("opt.schur_features")

			return true;
		} // end solve()


		void realize_relinearized()
		{
			DETAILED_PROFILING_ENTER("opt.schur_realize_HAp_changed")
			// Update the starting value of HAp for Schur:
			schur_compl.realize_HAp_changed();
			DETAILED_PROFILING_LEAVE("opt.schur_realize_HAp_changed")
		}
		void realize_lambda_changed()
		{
			// Nothing to do.
		}
		bool was_ith_feature_invertible(const size_t i)
		{
			return schur_compl.was_ith_feature_invertible(i);
		}
		/** Here, out_info is of type mrpt::srba::options::solver_LM_no_schur_sparse_cholesky::extra_results_t */
		void get_extra_results(typename RBA_ENGINE::rba_options_type::solver_t::extra_results_t & out_info )
		{
			out_info.hessian_valid = sS_is_valid;
			if (sS) sS->swap(out_info.hessian);
		}
	};

	// ------------------------------------------------------------------------------------------
	/** SOLVER: Lev-Marq with Schur, with Sparse Cholesky (CSparse library) */
	template <class RBA_ENGINE>
	struct solver_engine<true /*Schur*/,true /*dense Chol*/,RBA_ENGINE>
	{
		typedef typename RBA_ENGINE::hessian_traits_t hessian_traits_t;

		static const size_t POSE_DIMS = RBA_ENGINE::kf2kf_pose_type::REL_POSE_DIMS;
		static const size_t LM_DIMS   = RBA_ENGINE::lm_type::LM_DIMS;

		const int m_verbose_level;
		mrpt::utils::CTimeLogger &m_profiler;
		const size_t nUnknowns_k2k, nUnknowns_k2f;

		vector_double  delta_eps; //!< The result of solving Ax=b will be stored here
		typename hessian_traits_t::TSparseBlocksHessian_Ap  &HAp;
		typename hessian_traits_t::TSparseBlocksHessian_f   &Hf;
		typename hessian_traits_t::TSparseBlocksHessian_Apf &HApf;
		vector_double  &minus_grad;

		SchurComplement<
			typename hessian_traits_t::TSparseBlocksHessian_Ap,
			typename hessian_traits_t::TSparseBlocksHessian_f,
			typename hessian_traits_t::TSparseBlocksHessian_Apf
			>
			schur_compl;


		// Data for dense cholesky:
		Eigen::MatrixXd  denseH;
		Eigen::LLT<Eigen::MatrixXd,Eigen::Upper>  denseChol;
		bool  denseChol_is_uptodate;
		bool  hessian_is_valid; //!< Whether the hessian was correctly evaluated at least once.


		/** Constructor */
		solver_engine(
			const int verbose_level,
			mrpt::utils::CTimeLogger & profiler,
			typename hessian_traits_t::TSparseBlocksHessian_Ap  &HAp_,
			typename hessian_traits_t::TSparseBlocksHessian_f   &Hf_,
			typename hessian_traits_t::TSparseBlocksHessian_Apf &HApf_,
			vector_double  &minus_grad_,
			const size_t nUnknowns_k2k_,
			const size_t nUnknowns_k2f_) :
				m_verbose_level(verbose_level),
				m_profiler(profiler),
				nUnknowns_k2k(nUnknowns_k2k_),
				nUnknowns_k2f(nUnknowns_k2f_),
				HAp(HAp_),Hf(Hf_),HApf(HApf_),
				minus_grad(minus_grad_),
				schur_compl(
					HAp_,Hf_,HApf_, // The different symbolic/numeric Hessians
					&minus_grad[0],  // minus gradient of the Ap part
					// Handle case of no unknown features:
					nUnknowns_k2f!=0 ? &minus_grad[POSE_DIMS*nUnknowns_k2k] : NULL   // minus gradient of the features part
					),
				denseChol_is_uptodate (false),
				hessian_is_valid (false)
		{
		}

		// ----------------------------------------------------------------------
		// Solve the H·Ax = -g system using the Schur complement to generate a
		//   Ap-only reduced system.
		// Return: always true (success).
		// ----------------------------------------------------------------------
		bool solve(const double lambda)
		{
			// 1st: Numeric part: Update HAp hessian into the reduced system
			// Note: We have to re-evaluate the entire reduced Hessian HAp even if
			//       only lambda changed, because of the terms inv(Hf+\lambda*I).

			DETAILED_PROFILING_ENTER("opt.schur_build_reduced")
			schur_compl.numeric_build_reduced_system(lambda);
			DETAILED_PROFILING_LEAVE("opt.schur_build_reduced")

			if (schur_compl.getNumFeaturesFullRank()!=schur_compl.getNumFeatures())
				VERBOSE_LEVEL(1) << "[OPT] Schur warning: only " << schur_compl.getNumFeaturesFullRank() << " out of " << schur_compl.getNumFeatures() << " features have full-rank.\n";

			if (!denseChol_is_uptodate)
			{
				// Use a dense Cholesky method for solving the set of unknowns:
				denseH.setZero(nUnknowns_k2k*POSE_DIMS,nUnknowns_k2k*POSE_DIMS);  // Only for the H_Ap part of the Hessian
				hessian_is_valid = false;

				// Now write the updated "HAp" into its sparse matrix form:
				DETAILED_PROFILING_ENTER("opt.DenseFill")
				for (size_t i=0;i<nUnknowns_k2k;i++)
				{	// Only upper-half triangle:
					const typename hessian_traits_t::TSparseBlocksHessian_Ap::col_t & col_i = HAp.getCol(i);

					for (typename hessian_traits_t::TSparseBlocksHessian_Ap::col_t::const_iterator itRowEntry = col_i.begin();itRowEntry != col_i.end(); ++itRowEntry )
					{
						if (itRowEntry->first==i)
						{
							// block Diagonal: Add lambda*I to these ones
							typename hessian_traits_t::TSparseBlocksHessian_Ap::matrix_t sSii = itRowEntry->second.num;
							for (size_t k=0;k<POSE_DIMS;k++)
								sSii.coeffRef(k,k)+=lambda;
							denseH.block<POSE_DIMS,POSE_DIMS>(POSE_DIMS*i,POSE_DIMS*i) = sSii;
						}
						else
						{
							denseH.block<POSE_DIMS,POSE_DIMS>(
								POSE_DIMS*itRowEntry->first,  // row index
								POSE_DIMS*i)                   // col index
								= itRowEntry->second.num;
						}
					}
				}
				DETAILED_PROFILING_LEAVE("opt.DenseFill")

				hessian_is_valid = true;

				// Dense cholesky:
				// ----------------------------------
				DETAILED_PROFILING_ENTER("opt.DenseChol")
				denseChol = denseH.selfadjointView<Eigen::Upper>().llt();
				DETAILED_PROFILING_LEAVE("opt.DenseChol")

				if (denseChol.info()!=Eigen::Success)
				{
					// not positive definite so increase lambda and try again
					return false;
				}

				denseChol_is_uptodate = true;
			}


			// backsubtitution gives us "DeltaEps" from Cholesky and "-grad":
			//
			//    (J^tJ + lambda*I) DeltaEps = -grad
			// ----------------------------------------------------------------
			DETAILED_PROFILING_ENTER("opt.backsub")

			delta_eps.resize(nUnknowns_k2k*POSE_DIMS + nUnknowns_k2f*LM_DIMS );
			delta_eps.tail(nUnknowns_k2f*LM_DIMS).setZero();

			delta_eps.head(nUnknowns_k2k*POSE_DIMS) = denseChol.solve( minus_grad.head(nUnknowns_k2k*POSE_DIMS) );

			DETAILED_PROFILING_LEAVE("opt.backsub")

			// If using the Schur complement, at this point we must now solve the secondary
			//  system for features:
			// -----------------------------------------------------------------------------
			// 2nd numeric part: Solve for increments in features ----------
			DETAILED_PROFILING_ENTER("opt.schur_features")

			schur_compl.numeric_solve_for_features(
				&delta_eps[0],
				// Handle case of no unknown features:
				nUnknowns_k2f!=0 ? &delta_eps[nUnknowns_k2k*POSE_DIMS] : NULL   // minus gradient of the features part
				);

			DETAILED_PROFILING_LEAVE("opt.schur_features")

			return true;
		} // end solve()

		void realize_lambda_changed()
		{
			denseChol_is_uptodate = false;
		}
		void realize_relinearized()
		{
			DETAILED_PROFILING_ENTER("opt.schur_realize_HAp_changed")
			// Update the starting value of HAp for Schur:
			schur_compl.realize_HAp_changed();
			DETAILED_PROFILING_LEAVE("opt.schur_realize_HAp_changed")
		}
		bool was_ith_feature_invertible(const size_t i)
		{
			return schur_compl.was_ith_feature_invertible(i);
		}
		/** Here, out_info is of type mrpt::srba::options::solver_LM_schur_dense_cholesky::extra_results_t */
		void get_extra_results(typename RBA_ENGINE::rba_options_type::solver_t::extra_results_t & out_info )
		{
			out_info.hessian_valid = hessian_is_valid;
			denseH.swap(out_info.hessian);
		}
	};

} // end namespace "internal"

} }  // end namespaces
