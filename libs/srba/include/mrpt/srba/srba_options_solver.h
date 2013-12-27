/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/math/CSparseMatrix.h>

namespace mrpt { namespace srba {
namespace options
{

	/** \defgroup mrpt_srba_options_solver Types for RBA_OPTIONS::solver_t 
		* \ingroup mrpt_srba_options */

		/** Usage: A possible type for RBA_OPTIONS::solver_t.
		  * Meaning: Levenberg-Marquardt solver, Schur complement to reduce landmarks, dense Cholesky solver for Ax=b.
		  * \ingroup mrpt_srba_options_solver */
		struct solver_LM_schur_dense_cholesky
		{
			static const bool USE_SCHUR      = true;
			static const bool DENSE_CHOLESKY = true;
			/** Extra output information to be found in RbaEngine<>::TOptimizeExtraOutputInfo::extra_results */
			struct extra_results_t
			{
				bool hessian_valid; //!< Will be false if the Hessian wasn't evaluated for some reason.
				/** The last value of the (dense) Hessian (inverse of covariances) of all kf-to-kf relative pose unknowns (note that this is after Schur reduction) */
				Eigen::MatrixXd  hessian;

				extra_results_t() { clear(); }
				void clear() { hessian_valid=false; }
			};
		};

		/** Usage: A possible type for RBA_OPTIONS::solver_t.
		  * Meaning: Levenberg-Marquardt solver, Schur complement to reduce landmarks, sparse Cholesky solver for Ax=b.
		  * \ingroup mrpt_srba_options_solver */
		struct solver_LM_schur_sparse_cholesky
		{
			static const bool USE_SCHUR      = true;
			static const bool DENSE_CHOLESKY = false;
			/** Extra output information to be found in RbaEngine<>::TOptimizeExtraOutputInfo::extra_results */
			struct extra_results_t
			{
				bool hessian_valid; //!< Will be false if the Hessian wasn't evaluated for some reason.
				/** The column-compressed form of the last Hessian matrix (the inverse of covariance), for all the kf-to-kf unknowns (note that this is after Schur reduction) */
				mrpt::utils::CSparseMatrix  hessian;

				extra_results_t() { clear(); }
				void clear() { hessian_valid=false; }
			};
		};

		/** Usage: A possible type for RBA_OPTIONS::solver_t.
		  * Meaning: Levenberg-Marquardt solver, without Schur complement, sparse Cholesky solver for Ax=b.
		  * \ingroup mrpt_srba_options_solver */
		struct solver_LM_no_schur_sparse_cholesky
		{
			static const bool USE_SCHUR      = false;
			static const bool DENSE_CHOLESKY = false;
			/** Extra output information to be found in RbaEngine<>::TOptimizeExtraOutputInfo::extra_results */
			struct extra_results_t
			{
				bool hessian_valid; //!< Will be false if the Hessian wasn't evaluated for some reason.
				/** The column-compressed form of the last Hessian matrix (the inverse of covariance), for all the problem unknowns */
				mrpt::utils::CSparseMatrix  hessian;

				extra_results_t() { clear(); }
				void clear() { hessian_valid=false; }
			};
		};

} } } // End of namespaces
