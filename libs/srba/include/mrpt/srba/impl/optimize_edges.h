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

// Build flags:
#ifndef SRBA_SOLVE_USING_SCHUR_COMPLEMENT
#	define SRBA_SOLVE_USING_SCHUR_COMPLEMENT  1
#endif
#ifndef SRBA_USE_DENSE_CHOLESKY
#	define SRBA_USE_DENSE_CHOLESKY            1
#endif
#ifndef SRBA_DETAILED_TIME_PROFILING
#	define SRBA_DETAILED_TIME_PROFILING       0          //  Enabling this has a measurable impact in performance, so use only for debugging.
#endif

// Macros:
#if SRBA_DETAILED_TIME_PROFILING
#	define DETAILED_PROFILING_ENTER(_STR) m_profiler.enter(_STR);
#	define DETAILED_PROFILING_LEAVE(_STR) m_profiler.leave(_STR);
#else
#	define DETAILED_PROFILING_ENTER(_STR)
#	define DETAILED_PROFILING_LEAVE(_STR)
#endif

// ------------------------------------------
//         optimize_edges
//          (See header for docs)
// ------------------------------------------
template <class KF2KF_POSE_TYPE,class LM_TYPE,class OBS_TYPE,class RBA_OPTIONS>
void RBA_Problem<KF2KF_POSE_TYPE,LM_TYPE,OBS_TYPE,RBA_OPTIONS>::optimize_edges(
	const std::vector<size_t> & run_k2k_edges_in,
	const std::vector<size_t> & run_k2f_edges_in,
	TOptimizeExtraOutputInfo & out_info,
	const std::vector<size_t> & in_observation_indices_to_optimize
	)
{
	m_profiler.enter("opt");

	// Problem dimensions:
	const size_t POSE_DIMS = KF2KF_POSE_TYPE::REL_POSE_DIMS;
	const size_t LM_DIMS   = LM_TYPE::LM_DIMS;
	const size_t OBS_DIMS  = OBS_TYPE::OBS_DIMS;


	// Filter out those unknowns which for sure do not have any observation,
	//  which can be checked by looking for empty columns in the sparse Jacobian.
	// -------------------------------------------------------------------------------
	DETAILED_PROFILING_ENTER("opt.filter_unknowns")

	std::vector<size_t> run_k2k_edges; run_k2k_edges.reserve(run_k2k_edges_in.size());
	std::vector<size_t> run_k2f_edges; run_k2f_edges.reserve(run_k2f_edges_in.size());

	for (size_t i=0;i<run_k2k_edges_in.size();i++)
	{
		const typename TSparseBlocksJacobians_dh_dAp::col_t & col_i = rba_state.lin_system.dh_dAp.getCol( run_k2k_edges_in[i] );
		if (!col_i.empty()) run_k2k_edges.push_back( run_k2k_edges_in[i] );
		else std::cerr << "[RBA_Problem::optimize_edges] *Warning*: Skipping optimization of k2k edge #"<<run_k2k_edges_in[i] << " (" <<rba_state.k2k_edges[run_k2k_edges_in[i]].from <<"->"<<rba_state.k2k_edges[run_k2k_edges_in[i]].to <<") since no observation depends on it.\n";
	}

	const mrpt::utils::map_as_vector<size_t,size_t> & dh_df_remap = rba_state.lin_system.dh_df.getColInverseRemappedIndices();
	for (size_t i=0;i<run_k2f_edges_in.size();i++)
	{
		const TLandmarkID feat_id = run_k2f_edges_in[i];
		ASSERT_(feat_id<rba_state.all_lms.size())

		const typename rba_problem_state_t::TLandmarkEntry &lm_e = rba_state.all_lms[feat_id];
		ASSERTMSG_(lm_e.rfp!=NULL, "Trying to optimize an unknown feature ID")
		ASSERTMSG_(!lm_e.has_known_pos,"Trying to optimize a feature with fixed (known) value")

		mrpt::utils::map_as_vector<size_t,size_t>::const_iterator it_remap = dh_df_remap.find(feat_id);   // O(1) with map_as_vector
		ASSERT_(it_remap != dh_df_remap.end())

		const typename TSparseBlocksJacobians_dh_df::col_t & col_i = rba_state.lin_system.dh_df.getCol( it_remap->second );

		if (!col_i.empty()) run_k2f_edges.push_back( run_k2f_edges_in[i] );
		else { std::cerr << "[RBA_Problem::optimize_edges] *Warning*: Skipping optimization of k2f edge #"<<run_k2f_edges_in[i] << " since no observation depends on it.\n"; }
	}

	DETAILED_PROFILING_LEAVE("opt.filter_unknowns")

	// Build list of unknowns, and their corresponding columns in the Sparse Jacobian:
	// -------------------------------------------------------------------------------
	const size_t nUnknowns_k2k = run_k2k_edges.size();
	const size_t nUnknowns_k2f = run_k2f_edges.size();

	const size_t idx_start_f = POSE_DIMS*nUnknowns_k2k; // In the vector of unknowns, the 0-based first index of the first feature variable (before that, all are SE(3) edges)
	const size_t nUnknowns_scalars = POSE_DIMS*nUnknowns_k2k + LM_DIMS*nUnknowns_k2f;
	ASSERT_(nUnknowns_scalars>=1)

	// k2k edges:
	vector<typename TSparseBlocksJacobians_dh_dAp::col_t*>  dh_dAp(nUnknowns_k2k);
	vector<k2k_edge_t *> k2k_edge_unknowns(nUnknowns_k2k);
	for (size_t i=0;i<nUnknowns_k2k;i++)
	{
		dh_dAp[i] = &rba_state.lin_system.dh_dAp.getCol( run_k2k_edges[i] );
		k2k_edge_unknowns[i] = & rba_state.k2k_edges[run_k2k_edges[i]];
	}

	// k2f edges:
	vector<typename TSparseBlocksJacobians_dh_df::col_t*>  dh_df(nUnknowns_k2f);
	vector<TRelativeLandmarkPos*> k2f_edge_unknowns(nUnknowns_k2f);
	for (size_t i=0;i<nUnknowns_k2f;i++)
	{
		const TLandmarkID feat_id = run_k2f_edges[i];
		const typename rba_problem_state_t::TLandmarkEntry &lm_e = rba_state.all_lms[feat_id];

		mrpt::utils::map_as_vector<size_t,size_t>::const_iterator it_remap = dh_df_remap.find(feat_id);  // O(1) with map_as_vector
		ASSERT_(it_remap != dh_df_remap.end())

		dh_df[i] = &rba_state.lin_system.dh_df.getCol( it_remap->second );
		k2f_edge_unknowns[i] = lm_e.rfp;
	}

	// Unless stated otherwise, take into account ALL the observations involved in each
	// unknown (i.e. don't discard any information).
	// -------------------------------------------------------------------------------
	DETAILED_PROFILING_ENTER("opt.build_obs_list")


	// Clear the entries in "m_cached_obs_global_idx2residual_idx" marked as used in the last step in "m_used_indices_in_obs_map".
	// This may seem double work, but in this way the size of "m_used_indices_in_obs_map" is bounded, while the unbounded "obs_global_idx2residual_idx"
	// is reused over time, avoiding O(n) memory allocations ;-)
	//for (size_t i=0;i<m_used_indices_in_obs_map.size();i++) m_cached_obs_global_idx2residual_idx[i].valid=false;
	//m_used_indices_in_obs_map.clear();

	// Mapping betwwen obs. indices in the residuals vector & the global list of all the observations:
	std::map<size_t,size_t>   obs_global_idx2residual_idx; //--> m_cached_obs_global_idx2residual_idx

	std::vector<TObsUsed>  involved_obs;  //  + obs. data
	if (in_observation_indices_to_optimize.empty())
	{
		// Observations for k2k edges Jacobians
		for (size_t i=0;i<nUnknowns_k2k;i++)
		{
			// For each column, process each nonzero block:
			typename TSparseBlocksJacobians_dh_dAp::col_t *col = dh_dAp[i];

			for (typename TSparseBlocksJacobians_dh_dAp::col_t::iterator it=col->begin();it!=col->end();++it)
			{
				const size_t global_obs_idx = it->first;
				const size_t obs_idx = involved_obs.size();

				obs_global_idx2residual_idx[global_obs_idx] = obs_idx;

				//m_used_indices_in_obs_map.push_back(global_obs_idx);

				involved_obs.push_back( TObsUsed (global_obs_idx, &rba_state.all_observations[global_obs_idx] ) );
			}
		}
		// Observations for k2f edges Jacobians
		for (size_t i=0;i<nUnknowns_k2f;i++)
		{
			// For each column, process each nonzero block:
			typename TSparseBlocksJacobians_dh_df::col_t *col = dh_df[i];

			for (typename TSparseBlocksJacobians_dh_df::col_t::iterator it=col->begin();it!=col->end();++it)
			{
				const size_t global_obs_idx = it->first;
				// Only add if not already added before:
				std::map<size_t,size_t>::const_iterator it_o = obs_global_idx2residual_idx.find(global_obs_idx);
				/*	TSizeFlag & sf = obs_global_idx2residual_idx[global_obs_idx];*/
				if (it_o == obs_global_idx2residual_idx.end())
				{
					const size_t obs_idx = involved_obs.size();

					obs_global_idx2residual_idx[global_obs_idx] = obs_idx;

					//m_used_indices_in_obs_map.push_back(global_obs_idx);

					involved_obs.push_back( TObsUsed( global_obs_idx,  &rba_state.all_observations[global_obs_idx] ) );
				}
			}
		}
	}
	else
	{
		// use only those observations in "in_observation_indices_to_optimize":
		const size_t nInObs = in_observation_indices_to_optimize.size();
		for (size_t i=0;i<nInObs;i++)
		{
			const size_t global_obs_idx = in_observation_indices_to_optimize[i];
			const size_t obs_idx = involved_obs.size();

			//TSizeFlag & sf = obs_global_idx2residual_idx[global_obs_idx];
			//m_used_indices_in_obs_map.push_back(global_obs_idx);
			//sf.value = obs_idx;
			//sf.valid = true;
			obs_global_idx2residual_idx[global_obs_idx] = obs_idx;

			involved_obs.push_back(TObsUsed(global_obs_idx, &rba_state.all_observations[global_obs_idx] ) );
		}
	}

	DETAILED_PROFILING_LEAVE("opt.build_obs_list")

	const size_t nObs = involved_obs.size();


	// Make a list of Keyframe IDs whose numeric spanning trees must be updated (so we only update them!)
	// -------------------------------------------------------------------------------
	DETAILED_PROFILING_ENTER("opt.prep_list_num_tree_roots")

	std::set<TKeyFrameID>  kfs_num_spantrees_to_update;
	prepare_Jacobians_required_tree_roots(kfs_num_spantrees_to_update, dh_dAp, dh_df);

	DETAILED_PROFILING_LEAVE("opt.prep_list_num_tree_roots")

	VERBOSE_LEVEL(2) << "[OPT] KF roots whose spantrees need numeric-updates: " << srba::sprintf_container("%u", kfs_num_spantrees_to_update) <<endl;


	// Spanning tree: Update numerically only those entries which we really need:
	// -------------------------------------------------------------------------------
	DETAILED_PROFILING_ENTER("opt.update_spanning_tree_num")
	const size_t count_span_tree_num_update = rba_state.spanning_tree.update_numeric(kfs_num_spantrees_to_update, false /* don't skip those marked as updated, so update all */);
	DETAILED_PROFILING_LEAVE("opt.update_spanning_tree_num")


	// Re-evaluate all Jacobians numerically:
	// -------------------------------------------------------------------------------
	std::vector<const pose_flag_t*>    list_of_required_num_poses; // Filled-in by Jacobian evaluation upon first call.


	DETAILED_PROFILING_ENTER("opt.reset_Jacobs_validity")
	// Before evaluating Jacobians we must reset as "valid" all the involved observations.
	//  If needed, they'll be marked as invalid by the Jacobian evaluator if just one of the components
	//  for one observation leads to an error.
	for (size_t i=0;i<nObs;i++)
		rba_state.all_observations_Jacob_validity[ involved_obs[i].obs_idx ] = 1;

	DETAILED_PROFILING_LEAVE("opt.reset_Jacobs_validity")


	DETAILED_PROFILING_ENTER("opt.recompute_all_Jacobians")
	const size_t count_jacobians = recompute_all_Jacobians(dh_dAp, dh_df, &list_of_required_num_poses );
	DETAILED_PROFILING_LEAVE("opt.recompute_all_Jacobians")

	// Mark all required spanning-tree numeric entries as outdated, so an exception will reveal us if
	//  next time Jacobians are required they haven't been updated as they should:
	// -------------------------------------------------------------------------------
	for (size_t i=0;i<list_of_required_num_poses.size();i++)
		list_of_required_num_poses[i]->mark_outdated();



#if 0
	{
		CMatrixDouble Jbin;
		rba_state.lin_system.dh_dAp.getBinaryBlocksRepresentation(Jbin);
		Jbin.saveToTextFile("sparse_jacobs_blocks.txt", mrpt::math::MATRIX_FORMAT_INT );
		rba_state.lin_system.dh_dAp.saveToTextFileAsDense("sparse_jacobs.txt");
		mrpt::system::pause();
	}
#endif

	// ----------------------------------------------------------------------
	//  Compute H = J^t * J , exploting the sparse, block representation of J:
	//
	//   Don't add the LevMarq's "\lambda*I"  here so "H" needs not to be
	//    modified between different LevMarq. trials with different lambda values.
	// ----------------------------------------------------------------------
	typename hessian_traits_t::TSparseBlocksHessian_Ap  HAp;
	typename hessian_traits_t::TSparseBlocksHessian_f   Hf;
	typename hessian_traits_t::TSparseBlocksHessian_Apf HApf;

	// This symbolic constructions must be done only ONCE:
	DETAILED_PROFILING_ENTER("opt.sparse_hessian_build_symbolic")
	sparse_hessian_build_symbolic(
		HAp,Hf,HApf,
		dh_dAp,dh_df
		);
	DETAILED_PROFILING_LEAVE("opt.sparse_hessian_build_symbolic")

	// and then we only have to do a numeric evaluation upon changes:
	size_t nInvalidJacobs = 0;
	DETAILED_PROFILING_ENTER("opt.sparse_hessian_update_numeric")
	nInvalidJacobs += sparse_hessian_update_numeric(HAp);
	nInvalidJacobs += sparse_hessian_update_numeric(Hf);
	nInvalidJacobs += sparse_hessian_update_numeric(HApf);
	DETAILED_PROFILING_LEAVE("opt.sparse_hessian_update_numeric")

	if (nInvalidJacobs)
		VERBOSE_LEVEL(1) << "[OPT] " << nInvalidJacobs << " Jacobian blocks ignored for 'invalid'.\n";

	VERBOSE_LEVEL(2) << "[OPT] Individual Jacobs: " << count_jacobians << " #k2k_edges=" << nUnknowns_k2k << " #k2f_edges=" << nUnknowns_k2f << " #obs=" << nObs << endl;
	VERBOSE_LEVEL(2) << "[OPT] k2k_edges to optimize: " << sprintf_container("% u",run_k2k_edges) << endl;
	VERBOSE_LEVEL(2) << "[OPT] k2f_edges to optimize: " << sprintf_container("% u",run_k2f_edges) << endl;

	// VERY IMPORTANT: For J^t*J to be invertible, we need a full rank Hessian:
	//    nObs*OBS_DIMS >= nUnknowns_k2k*POSE_DIMS+nUnknowns_k2f*LM_DIMS
	//
	ASSERT_ABOVEEQ_(OBS_DIMS*nObs,POSE_DIMS*nUnknowns_k2k+LM_DIMS*nUnknowns_k2f)

#if 0
	{
		static int cnt=0;
		HAp.saveToTextFileAsDense(mrpt::format("debug_Hessian_%05i.txt",++cnt));
	}
#endif

	// Extra params:
	const size_t max_iters            = this->parameters.srba.max_iters;
	const double max_error_per_obs_to_stop = this->parameters.srba.max_error_per_obs_to_stop;


	// Cholesky object, as a pointer to reuse it between iterations:
	std::auto_ptr<CSparseMatrix::CholeskyDecomp>  ptrCh;

	// ----------------------------------------------------------------
	//         Iterative Levenberg Marquardt (LM) algorithm
	//
	//   For a description, see:
	//    http://www.mrpt.org/Levenberg%E2%80%93Marquardt_algorithm
	// ----------------------------------------------------------------

	// LevMar parameters:
	double nu = 2;
	const double eps = 1e-16;
	double lambda = -1;  // initial value. <0 = auto.

	// Automatic guess of "lambda" = tau * max(diag(Hessian))   (Hessian=H here)
	if (lambda<0)
	{
		DETAILED_PROFILING_ENTER("opt.guess lambda")

		double Hess_diag_max = 0;
		for (size_t i=0;i<nUnknowns_k2k;i++)
		{
			ASSERTDEB_(HAp.getCol(i).find(i)!=HAp.getCol(i).end())

			const double Hii_max = HAp.getCol(i)[i].num.diagonal().maxCoeff();
			mrpt::utils::keep_max(Hess_diag_max, Hii_max);
		}
		for (size_t i=0;i<nUnknowns_k2f;i++)
		{
			ASSERTDEB_(Hf.getCol(i).find(i)!=Hf.getCol(i).end())

			const double Hii_max = Hf.getCol(i)[i].num.diagonal().maxCoeff();
			mrpt::utils::keep_max(Hess_diag_max, Hii_max);
		}

		const double tau = 1e-3;
		lambda = tau * Hess_diag_max;

		DETAILED_PROFILING_LEAVE("opt.guess lambda")
	}

	// Compute the reprojection errors:
	//  residuals = "h(x)-z" (a vector of 2-vectors).
	// ---------------------------------------------------------------------------------
	vector_residuals_t  residuals(nObs);

	DETAILED_PROFILING_ENTER("opt.reprojection_residuals")
	double total_proj_error = reprojection_residuals(
		residuals, // Out
		involved_obs // In
		);
	DETAILED_PROFILING_LEAVE("opt.reprojection_residuals")

	double proj_error_per_obs_px = std::sqrt(total_proj_error/nObs);

	out_info.num_observations     = nObs;
	out_info.num_jacobians        = count_jacobians;
	out_info.num_kf2kf_edges_optimized = run_k2k_edges.size();
	out_info.num_kf2lm_edges_optimized = run_k2f_edges.size();
	out_info.num_total_scalar_optimized = nUnknowns_scalars;
	out_info.num_span_tree_numeric_updates = count_span_tree_num_update;
	out_info.total_sqr_error_init = total_proj_error;


	VERBOSE_LEVEL(1) << "[OPT] LM: Initial avr. err in px=" <<  proj_error_per_obs_px << " #Jcbs=" << count_jacobians << " #k2k_edges=" << nUnknowns_k2k << " #k2f_edges=" << nUnknowns_k2f << " #obs=" << nObs << endl;

	if (parameters.srba.feedback_user_iteration)
		(*parameters.srba.feedback_user_iteration)(0,total_proj_error,proj_error_per_obs_px);

	// Compute the gradient: "grad = J^t * (h(x)-z)"
	// ---------------------------------------------------------------------------------
	// The indices of observations in the residuals vector, in the same order than we go thru them in compute_minus_gradient()
	//std::vector<size_t> sequential_obs_indices;

	//DETAILED_PROFILING_ENTER("opt.build_obs_indices")
	//build_obs_indices(/* Out: */ sequential_obs_indices, /* In: */ dh_dAp, dh_df, m_cached_obs_global_idx2residual_idx);
	//DETAILED_PROFILING_LEAVE("opt.build_obs_indices")

	vector_double  minus_grad; // The negative of the gradient.

	DETAILED_PROFILING_ENTER("opt.compute_minus_gradient")
	compute_minus_gradient(/* Out: */ minus_grad, /* In: */ dh_dAp, dh_df, residuals, obs_global_idx2residual_idx); //sequential_obs_indices);
	DETAILED_PROFILING_LEAVE("opt.compute_minus_gradient")


	// Build symbolic structures for Schur complement:
	// ---------------------------------------------------------------------------------
#if SRBA_SOLVE_USING_SCHUR_COMPLEMENT
	DETAILED_PROFILING_ENTER("opt.schur_compl_build_symbolic")

	SchurComplement<
		typename hessian_traits_t::TSparseBlocksHessian_Ap,
		typename hessian_traits_t::TSparseBlocksHessian_f,
		typename hessian_traits_t::TSparseBlocksHessian_Apf
		>
		schur_compl(
			HAp,Hf,HApf, // The different symbolic/numeric Hessian
			&minus_grad[0],  // minus gradient of the Ap part
			// Handle case of no unknown features:
			nUnknowns_k2f!=0 ? &minus_grad[idx_start_f] : NULL   // minus gradient of the features part
			);

	DETAILED_PROFILING_LEAVE("opt.schur_compl_build_symbolic")

#endif

	const double MAX_LAMBDA = 1e20;

	// These are defined here to avoid allocatin/deallocating memory with each iteration:
	vector<k2k_edge_t>                old_k2k_edge_unknowns;
	vector<TRelativeLandmarkPos>      old_k2f_edge_unknowns;
	//srba::TRelativePosesForEachTarget old_span_tree;
	std::vector<pose_flag_t> 		old_span_tree; // In the same order than "list_of_required_num_poses"


	const std::string sLabelProfilerLM_iter = mrpt::format("opt.lm_iteration_k2k=%03u_k2f=%03u", static_cast<unsigned int>(nUnknowns_k2k), static_cast<unsigned int>(nUnknowns_k2f) );

	// LevMar iterations -------------------------------------
	size_t iter; // Declared here so we can read the final # of iterations out of the "for" loop.
	bool   stop = false;
	for (iter=0; iter<max_iters && !stop; iter++)
	{
		DETAILED_PROFILING_ENTER(sLabelProfilerLM_iter.c_str())

		// Try with different rho's until a better solution is found:
		double rho = 0;
		if (lambda>=MAX_LAMBDA)
		{
			stop=true;
			VERBOSE_LEVEL(2) << "[OPT] LM end criterion: lambda too large. " << lambda << ">=" <<MAX_LAMBDA<<endl;
		}
		if (proj_error_per_obs_px < max_error_per_obs_to_stop)
		{
			stop=true;
			VERBOSE_LEVEL(2) << "[OPT] LM end criterion: error too small. " << proj_error_per_obs_px << "<" <<max_error_per_obs_to_stop<<endl;
		}

		while(rho<=0 && !stop)
		{
			// -------------------------------------------------------------------------
			//  Build the sparse sS=(H+\lambda I) matrix to be decomposed with Cholesky
			// -------------------------------------------------------------------------

#if !SRBA_SOLVE_USING_SCHUR_COMPLEMENT
			// --------------------------------------------------------
			// Strategy #1: Solve the entire H Ax = -g system with
			//               H as a single sparse Hessian.
			// --------------------------------------------------------
			DETAILED_PROFILING_ENTER("opt.SparseTripletFill")

			CSparseMatrix sS(nUnknowns_scalars,nUnknowns_scalars);

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
						for (int k=0;k<POSE_DIMS;k++)
							sSii.coeffRef(k,k)+=lambda;
						sS.insert_submatrix(POSE_DIMS*i,POSE_DIMS*i, sSii );
					}
					else
					{
						sS.insert_submatrix(
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
					sS.insert_submatrix(
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
						for (int k=0;k<LM_DIMS;k++)
							sSii.coeffRef(k,k)+=lambda;
						sS.insert_submatrix(idx_start_f+LM_DIMS*i,idx_start_f+LM_DIMS*i, sSii );
					}
					else
					{
						sS.insert_submatrix(
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
			sS.compressFromTriplet();
			DETAILED_PROFILING_LEAVE("opt.SparseTripletCompress")

#if 0
			sS.saveToTextFile_dense("full_H.txt");
			minus_grad.saveToTextFile("full_minus_grad.txt");
#endif


			// Sparse cholesky
			// --------------------------------------
			DETAILED_PROFILING_ENTER("opt.SparseChol")
			try
			{
				if (!ptrCh.get())
						ptrCh = std::auto_ptr<CSparseMatrix::CholeskyDecomp>(new CSparseMatrix::CholeskyDecomp(sS) );
				else ptrCh.get()->update(sS);

				DETAILED_PROFILING_LEAVE("opt.SparseChol")
			}
			catch (CExceptionNotDefPos &)
			{
				DETAILED_PROFILING_LEAVE("opt.SparseChol")

				// not positive definite so increase lambda and try again
				lambda *= nu;
				nu *= 2.;
				stop = (lambda>MAX_LAMBDA);

				VERBOSE_LEVEL(2) << "[OPT] LM iter #"<< iter << " NotDefPos in Cholesky. Retrying with lambda=" << lambda << endl;
				continue;
			}

			// backsubtitution gives us "DeltaEps" from Cholesky and "-grad":
			//
			//    (J^tJ + lambda*I) DeltaEps = -grad
			// ----------------------------------------------------------------
			DETAILED_PROFILING_ENTER("opt.backsub")

			vector_double  delta_eps;
			ptrCh->backsub(minus_grad,delta_eps);

			DETAILED_PROFILING_LEAVE("opt.backsub")

#else // SRBA_SOLVE_USING_SCHUR_COMPLEMENT

			// --------------------------------------------------------
			// Strategy #2: Solve the H·Ax = -g system using the schur
			//               complement to generate a Ap-only reduced
			//               system.
			// --------------------------------------------------------

			// 1st: Numeric part: Update HAp hessian into the reduced system ----------
#if defined(DEBUG_DUMP_SCHUR_MATRICES)
	HAp.saveToTextFileAsDense("HAp.txt", true, true );
	Hf.saveToTextFileAsDense("Hf.txt", true, true);
	HApf.saveToTextFileAsDense("HApf.txt",false, false);
	minus_grad.saveToTextFile("minus_grad.txt");
	{ ofstream f("lambda.txt"); f << lambda << endl; }
#endif
			// Note: We have to re-evaluate the entire reduced Hessian HAp even if
			//       only lambda changed, because of the terms inv(Hf+\lambda*I).

			DETAILED_PROFILING_ENTER("opt.schur_build_reduced")

			schur_compl.numeric_build_reduced_system(lambda);

			DETAILED_PROFILING_LEAVE("opt.schur_build_reduced")

			if (schur_compl.getNumFeaturesFullRank()!=schur_compl.getNumFeatures())
				VERBOSE_LEVEL(1) << "[OPT] Schur: " << schur_compl.getNumFeaturesFullRank() << " out of " << schur_compl.getNumFeatures() << " features have full-rank.\n";

#if !SRBA_USE_DENSE_CHOLESKY
			CSparseMatrix sS(nUnknowns_k2k*POSE_DIMS,nUnknowns_k2k*POSE_DIMS);  // Only for the H_Ap part of the Hessian

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
						for (int k=0;k<POSE_DIMS;k++)
							sSii.coeffRef(k,k)+=lambda;
						sS.insert_submatrix(POSE_DIMS*i,POSE_DIMS*i, sSii );
					}
					else
					{
						sS.insert_submatrix(
							POSE_DIMS*itRowEntry->first,  // row index
							POSE_DIMS*i,                  // col index
							itRowEntry->second.num );
					}
				}
			}
			DETAILED_PROFILING_LEAVE("opt.SparseTripletFill")

#if defined(DEBUG_DUMP_SCHUR_MATRICES)
			sS.saveToTextFile_dense("schur_HAp.txt");
			minus_grad.saveToTextFile("schur_minus_grad_Ap.txt");
#endif

			// Compress the sparse matrix:
			// ----------------------------------
			DETAILED_PROFILING_ENTER("opt.SparseTripletCompress")
			sS.compressFromTriplet();
			DETAILED_PROFILING_LEAVE("opt.SparseTripletCompress")

			// Sparse cholesky:
			// ----------------------------------
			DETAILED_PROFILING_ENTER("opt.SparseChol")
			try
			{
				if (!ptrCh.get())
						ptrCh = std::auto_ptr<CSparseMatrix::CholeskyDecomp>(new CSparseMatrix::CholeskyDecomp(sS) );
				else ptrCh.get()->update(sS);

				DETAILED_PROFILING_LEAVE("opt.SparseChol")
			}
			catch (CExceptionNotDefPos &)
			{
				DETAILED_PROFILING_LEAVE("opt.SparseChol")

				// not positive definite so increase lambda and try again
				lambda *= nu;
				nu *= 2.;
				stop = (lambda>MAX_LAMBDA);

#ifdef _DEBUG
				{
					static int cnt=0;
					sS.saveToTextFile_dense(mrpt::format("_DEBUG_Hessian_error_cholesky_%03i.txt",cnt++));
				}
#endif

				VERBOSE_LEVEL(2) << "[OPT] LM iter #"<< iter << " NotDefPos in Cholesky. Retrying with lambda=" << lambda << endl;
				continue;
			}

			// backsubtitution gives us "DeltaEps" from Cholesky and "-grad":
			//
			//    (J^tJ + lambda*I) DeltaEps = -grad
			// ----------------------------------------------------------------
			DETAILED_PROFILING_ENTER("opt.backsub")

			vector_double  delta_eps(nUnknowns_k2k*POSE_DIMS + nUnknowns_k2f*LM_DIMS );  // Important: It's initialized to zeros!

			ptrCh->backsub(&minus_grad[0],&delta_eps[0],nUnknowns_k2k*POSE_DIMS);

			DETAILED_PROFILING_LEAVE("opt.backsub")

#else // SRBA_USE_DENSE_CHOLESKY

			// Use a dense Cholesky method for solving the set of unknowns:
			Eigen::MatrixXd  denseH(nUnknowns_k2k*POSE_DIMS,nUnknowns_k2k*POSE_DIMS);  // Only for the H_Ap part of the Hessian
			denseH.setZero();

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

#if defined(DEBUG_DUMP_SCHUR_MATRICES)
			denseH.saveToTextFile("schur_HAp.txt");
			minus_grad.saveToTextFile("schur_minus_grad_Ap.txt");
#endif

			// Dense cholesky:
			// ----------------------------------
			DETAILED_PROFILING_ENTER("opt.DenseChol")

			Eigen::LLT<Eigen::MatrixXd,Eigen::Upper> Chol = denseH.selfadjointView<Eigen::Upper>().llt();

			DETAILED_PROFILING_LEAVE("opt.DenseChol")

			if (Chol.info()!=Eigen::Success)
			{
				// not positive definite so increase lambda and try again
				lambda *= nu;
				nu *= 2.;
				stop = (lambda>MAX_LAMBDA);

				VERBOSE_LEVEL(2) << "[OPT] LM iter #"<< iter << " NotDefPos in Cholesky. Retrying with lambda=" << lambda << endl;
				continue;
			}


			// backsubtitution gives us "DeltaEps" from Cholesky and "-grad":
			//
			//    (J^tJ + lambda*I) DeltaEps = -grad
			// ----------------------------------------------------------------
			DETAILED_PROFILING_ENTER("opt.backsub")

			Eigen::VectorXd  delta_eps(nUnknowns_k2k*POSE_DIMS + nUnknowns_k2f*LM_DIMS );
			delta_eps.tail(nUnknowns_k2f*LM_DIMS).setZero();

			delta_eps.head(nUnknowns_k2k*POSE_DIMS) = Chol.solve( minus_grad.head(nUnknowns_k2k*POSE_DIMS) );

			DETAILED_PROFILING_LEAVE("opt.backsub")
#endif

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

#if defined(DEBUG_DUMP_SCHUR_MATRICES)
			delta_eps.saveToTextFile("deltas_Ap_f_schur.txt");
#endif

#endif // SRBA_SOLVE_USING_SCHUR_COMPLEMENT


			// Make a copy of the old edge values, just in case we need to restore them back...
			// ----------------------------------------------------------------------------------
			DETAILED_PROFILING_ENTER("opt.make_backup_copy_edges")

			old_k2k_edge_unknowns.resize(nUnknowns_k2k);
			for (size_t i=0;i<nUnknowns_k2k;i++)
				old_k2k_edge_unknowns[i] = *k2k_edge_unknowns[i];

			old_k2f_edge_unknowns.resize(nUnknowns_k2f);
			for (size_t i=0;i<nUnknowns_k2f;i++)
				old_k2f_edge_unknowns[i] = *k2f_edge_unknowns[i];

			DETAILED_PROFILING_LEAVE("opt.make_backup_copy_edges")

			// Add SE(2/3) deltas to the k2k edges:
			// ------------------------------------
			DETAILED_PROFILING_ENTER("opt.add_se3_deltas_to_frames")
			for (size_t i=0;i<nUnknowns_k2k;i++)
			{
				// edges_to_optimize:
				const pose_t &old_pose = k2k_edge_unknowns[i]->inv_pose;
				pose_t new_pose(UNINITIALIZED_POSE);

				// Use the Lie Algebra methods for the increment:
				const CArrayDouble<POSE_DIMS> incr( &delta_eps[POSE_DIMS*i] );
				pose_t  incrPose(UNINITIALIZED_POSE);
				se_traits_t::exp(incr,incrPose);   // incrPose = exp(incr) (Lie algebra exponential map)

				//new_pose =  old_pose  [+] delta
				//         = exp(delta) (+) old_pose
				new_pose.composeFrom(incrPose, old_pose);

				//  Overwrite problem graph:
				k2k_edge_unknowns[i]->inv_pose = new_pose;
			}
			DETAILED_PROFILING_LEAVE("opt.add_se3_deltas_to_frames")


			// Add R^3 deltas to the k2f edges:
			// ------------------------------------
			DETAILED_PROFILING_ENTER("opt.add_deltas_to_feats")
			for (size_t i=0;i<nUnknowns_k2f;i++)
			{
				const double *delta_feat = &delta_eps[idx_start_f+LM_DIMS*i];
				for (size_t k=0;k<LM_DIMS;k++)
					k2f_edge_unknowns[i]->pos[k] += delta_feat[k];
			}
			DETAILED_PROFILING_LEAVE("opt.add_deltas_to_feats")


			// Update the Spanning tree, making a back-up copy:
			// ------------------------------------------------------


			DETAILED_PROFILING_ENTER("opt.make_backup_copy_spntree_num")
			// DON'T: old_span_tree = rba_state.spanning_tree.num;  // This works but runs in O(n) with the size of the map!!
			// Instead: copy just the required entries:
			{
				const size_t nReqNumPoses = list_of_required_num_poses.size();
				if (old_span_tree.size()!=nReqNumPoses) old_span_tree.resize(nReqNumPoses);
				for (size_t i=0;i<nReqNumPoses;i++) old_span_tree[i].pose = list_of_required_num_poses[i]->pose;
			}
			DETAILED_PROFILING_LEAVE("opt.make_backup_copy_spntree_num")


			DETAILED_PROFILING_ENTER("opt.update_spanning_tree_num")
			for (size_t i=0;i<list_of_required_num_poses.size();i++)
				list_of_required_num_poses[i]->mark_outdated();

			rba_state.spanning_tree.update_numeric(kfs_num_spantrees_to_update, true /* Only those marked as outdated above */);
			DETAILED_PROFILING_LEAVE("opt.update_spanning_tree_num")

			// Compute new reprojection errors:
			// ----------------------------------
			vector_residuals_t  new_residuals;

			DETAILED_PROFILING_ENTER("opt.reprojection_residuals")
			double new_total_proj_error = reprojection_residuals(
				new_residuals, // Out
				involved_obs // In
				);
			DETAILED_PROFILING_LEAVE("opt.reprojection_residuals")

			double new_proj_error_per_obs_px = std::sqrt(new_total_proj_error/nObs);

			// is this better or worse?
			// -----------------------------
			rho = (total_proj_error - new_total_proj_error)/ (delta_eps.array()*(lambda*delta_eps + minus_grad).array() ).sum();

			if(rho>0)
			{
				// Good: Accept new values
				VERBOSE_LEVEL(2) << "[OPT] LM iter #"<< iter << " err: " << proj_error_per_obs_px << " -> " << new_proj_error_per_obs_px <<  "px, rho=" << rho << endl;
				if (parameters.srba.feedback_user_iteration)
					(*parameters.srba.feedback_user_iteration)(iter,new_total_proj_error,new_proj_error_per_obs_px);

				// Switch variables to the temptative ones, which are now accepted:
				//  (swap where possible, since it's faster)
				// ---------------------------------------------------------------------
				residuals.swap( new_residuals );

				total_proj_error      = new_total_proj_error;
				proj_error_per_obs_px = new_proj_error_per_obs_px;


				DETAILED_PROFILING_ENTER("opt.reset_Jacobs_validity")
				// Before evaluating Jacobians we must reset as "valid" all the involved observations.
				//  If needed, they'll be marked as invalid by the Jacobian evaluator if just one of the components
				//  for one observation leads to an error.
				for (size_t i=0;i<nObs;i++)
					rba_state.all_observations_Jacob_validity[ involved_obs[i].obs_idx ] = 1;

				DETAILED_PROFILING_LEAVE("opt.reset_Jacobs_validity")

				// Recalculate Jacobians:
				DETAILED_PROFILING_ENTER("opt.recompute_all_Jacobians")
				recompute_all_Jacobians(dh_dAp, dh_df);
				DETAILED_PROFILING_LEAVE("opt.recompute_all_Jacobians")

				// Recalculate Hessian:
				DETAILED_PROFILING_ENTER("opt.sparse_hessian_update_numeric")
				sparse_hessian_update_numeric(HAp);
				sparse_hessian_update_numeric(Hf);
				sparse_hessian_update_numeric(HApf);
				DETAILED_PROFILING_LEAVE("opt.sparse_hessian_update_numeric")

#if SRBA_SOLVE_USING_SCHUR_COMPLEMENT
				DETAILED_PROFILING_ENTER("opt.schur_realize_HAp_changed")

				// Update the starting value of HAp for Schur:
				schur_compl.realize_HAp_changed();

				DETAILED_PROFILING_LEAVE("opt.schur_realize_HAp_changed")
#endif

				// Update gradient:
				DETAILED_PROFILING_ENTER("opt.compute_minus_gradient")
				compute_minus_gradient(/* Out: */ minus_grad, /* In: */ dh_dAp, dh_df, residuals, obs_global_idx2residual_idx); //sequential_obs_indices);
				DETAILED_PROFILING_LEAVE("opt.compute_minus_gradient")

				// Reset other vars:
				stop = norm_inf(minus_grad)<=eps  || (proj_error_per_obs_px < max_error_per_obs_to_stop);
				lambda *= std::max(1.0/3.0, 1-std::pow(2*rho-1,3.0) );
				nu = 2.0;
			}
			else
			{
				DETAILED_PROFILING_ENTER("opt.failedstep_restore_backup")

				// Restore old values and retry again with a different lambda:
				//DON'T: rba_state.spanning_tree.num = old_span_tree; // NO! Don't do this, since existing pointers will break -> Copy elements one by one:
				{
					const size_t nReqNumPoses = list_of_required_num_poses.size();
					for (size_t i=0;i<nReqNumPoses;i++) const_cast<pose_flag_t*>(list_of_required_num_poses[i])->pose = old_span_tree[i].pose;
				}

				// Restore old edge values:
				for (size_t i=0;i<nUnknowns_k2k;i++)
					*k2k_edge_unknowns[i] = old_k2k_edge_unknowns[i];
				for (size_t i=0;i<nUnknowns_k2f;i++)
					*k2f_edge_unknowns[i] = old_k2f_edge_unknowns[i];

				DETAILED_PROFILING_LEAVE("opt.failedstep_restore_backup")

				VERBOSE_LEVEL(2) << "[OPT] LM iter #"<< iter << " no update,errs: " << sqrt(total_proj_error/nObs) << " < " << sqrt(new_total_proj_error/nObs) << " lambda=" << lambda <<endl;
				lambda *= nu;
				nu *= 2.0;
				stop = (lambda>1e9);
			}

		}; // end while rho

		DETAILED_PROFILING_LEAVE(sLabelProfilerLM_iter.c_str())

	} // end for LM "iter"


	// Final output info:
	out_info.total_sqr_error_final = total_proj_error;

	// Save the final information matrix of unknown features:
	DETAILED_PROFILING_ENTER("opt.get_Hf_diag_inv_cov")
	{
		ASSERT_(parameters.srba.std_noise_observations>0)
		const double inv_var_pixel_error = 1./parameters.srba.std_noise_observations;  // Scaling for information matrices below

		for (size_t i=0;i<nUnknowns_k2f;i++)
		{
#if SRBA_SOLVE_USING_SCHUR_COMPLEMENT
			if (!schur_compl.was_ith_feature_invertible(i))
				continue;
#endif

			const typename hessian_traits_t::TSparseBlocksHessian_f::col_t & col_i = Hf.getCol(i);
			ASSERT_(col_i.rbegin()->first==i)  // Make sure the last block matrix is the diagonal term of the upper-triangular matrix.

			const typename hessian_traits_t::TSparseBlocksHessian_f::matrix_t & inf_mat_src = col_i.rbegin()->second.num;
			typename hessian_traits_t::TSparseBlocksHessian_f::matrix_t & inf_mat_dst = rba_state.unknown_lms_inf_matrices[ run_k2f_edges[i] ];
			inf_mat_dst.noalias() = inf_mat_src * inv_var_pixel_error;
		}
	}
	DETAILED_PROFILING_LEAVE("opt.get_Hf_diag_inv_cov")

	if (parameters.srba.compute_condition_number)
	{
		DETAILED_PROFILING_ENTER("opt.condition_number")

		// Evaluate conditioning number of hessians:
		mrpt::math::CMatrixDouble dense_HAp;
		HAp.getAsDense(dense_HAp, true /* recover both triangular parts */);

		const Eigen::JacobiSVD<mrpt::math::CMatrixDouble::Base> H_svd = dense_HAp.jacobiSvd();
		out_info.HAp_condition_number = H_svd.singularValues().maxCoeff()/H_svd.singularValues().minCoeff();

		DETAILED_PROFILING_LEAVE("opt.condition_number")
	}

	m_profiler.leave("opt");

	VERBOSE_LEVEL(1) << "[OPT] ->LM: Final avr. err in px=" <<  proj_error_per_obs_px << " #iters=" << iter << "\n";
}


} }  // end namespaces
