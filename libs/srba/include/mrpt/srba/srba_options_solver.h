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
