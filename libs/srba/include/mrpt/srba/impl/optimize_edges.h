/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2015, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/math/ops_containers.h> // norm_inf()

namespace mrpt { namespace srba {

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

namespace internal
{
	/** Generic solver declaration */
	template <bool USE_SCHUR, bool DENSE_CHOL,class RBA_ENGINE>
	struct solver_engine;

	// Implemented in lev-marq_solvers.h
}


// ------------------------------------------
//         optimize_edges
//          (See header for docs)
// ------------------------------------------
template <class KF2KF_POSE_TYPE,class LM_TYPE,class OBS_TYPE,class RBA_OPTIONS>
void RbaEngine<KF2KF_POSE_TYPE,LM_TYPE,OBS_TYPE,RBA_OPTIONS>::optimize_edges(
	const std::vector<size_t> & run_k2k_edges_in,
	const std::vector<size_t> & run_feat_ids_in,
	TOptimizeExtraOutputInfo & out_info,
	const std::vector<size_t> & in_observation_indices_to_optimize
	)
{
	using namespace std;
	// This method deals with many common tasks to any optimizer: update Jacobians, prepare Hessians, etc. 
	// The specific solver method details are implemented in "my_solver_t":
	typedef internal::solver_engine<RBA_OPTIONS::solver_t::USE_SCHUR,RBA_OPTIONS::solver_t::DENSE_CHOLESKY,rba_engine_t> my_solver_t;
	
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
	std::vector<size_t> run_feat_ids; run_feat_ids.reserve(run_feat_ids_in.size());

	for (size_t i=0;i<run_k2k_edges_in.size();i++)
	{
		const typename TSparseBlocksJacobians_dh_dAp::col_t & col_i = rba_state.lin_system.dh_dAp.getCol( run_k2k_edges_in[i] );
		if (!col_i.empty()) run_k2k_edges.push_back( run_k2k_edges_in[i] );
		else 
		{
#ifdef SRBA_WORKAROUND_MSVC9_DEQUE_BUG
			const TKeyFrameID from = rba_state.k2k_edges[run_k2k_edges_in[i]]->from;
			const TKeyFrameID to   = rba_state.k2k_edges[run_k2k_edges_in[i]]->to;
#else
			const TKeyFrameID from = rba_state.k2k_edges[run_k2k_edges_in[i]].from;
			const TKeyFrameID to   = rba_state.k2k_edges[run_k2k_edges_in[i]].to;
#endif
			std::cerr << "[RbaEngine::optimize_edges] *Warning*: Skipping optimization of k2k edge #"<<run_k2k_edges_in[i] << " (" <<from <<"->"<<to <<") since no observation depends on it.\n";
		}
	}

	const mrpt::utils::map_as_vector<size_t,size_t> & dh_df_remap = rba_state.lin_system.dh_df.getColInverseRemappedIndices();
	for (size_t i=0;i<run_feat_ids_in.size();i++)
	{
		const TLandmarkID feat_id = run_feat_ids_in[i];
		ASSERT_(feat_id<rba_state.all_lms.size())

		const typename rba_problem_state_t::TLandmarkEntry &lm_e = rba_state.all_lms[feat_id];
		ASSERTMSG_(lm_e.rfp!=NULL, "Trying to optimize an unknown feature ID")
		ASSERTMSG_(!lm_e.has_known_pos,"Trying to optimize a feature with fixed (known) value")

		mrpt::utils::map_as_vector<size_t,size_t>::const_iterator it_remap = dh_df_remap.find(feat_id);   // O(1) with map_as_vector
		ASSERT_(it_remap != dh_df_remap.end())

		const typename TSparseBlocksJacobians_dh_df::col_t & col_i = rba_state.lin_system.dh_df.getCol( it_remap->second );

		if (!col_i.empty()) run_feat_ids.push_back( run_feat_ids_in[i] );
		else { std::cerr << "[RbaEngine::optimize_edges] *Warning*: Skipping optimization of k2f edge #"<<run_feat_ids_in[i] << " since no observation depends on it.\n"; }
	}

	DETAILED_PROFILING_LEAVE("opt.filter_unknowns")

	// Build list of unknowns, and their corresponding columns in the Sparse Jacobian:
	// -------------------------------------------------------------------------------
	const size_t nUnknowns_k2k = run_k2k_edges.size();
	const size_t nUnknowns_k2f = run_feat_ids.size();

	const size_t idx_start_f = POSE_DIMS*nUnknowns_k2k; // In the vector of unknowns, the 0-based first index of the first feature variable (before that, all are SE(3) edges)
	const size_t nUnknowns_scalars = POSE_DIMS*nUnknowns_k2k + LM_DIMS*nUnknowns_k2f;
	if (!nUnknowns_scalars)
	{
		std::cerr << "[RbaEngine::optimize_edges] *Warning*: Skipping optimization since no observation depends on any of the given variables.\n";
		out_info = TOptimizeExtraOutputInfo();
		return;
	}

	// k2k edges:
	std::vector<typename TSparseBlocksJacobians_dh_dAp::col_t*>  dh_dAp(nUnknowns_k2k);
	std::vector<k2k_edge_t *> k2k_edge_unknowns(nUnknowns_k2k);
	for (size_t i=0;i<nUnknowns_k2k;i++)
	{
		dh_dAp[i] = &rba_state.lin_system.dh_dAp.getCol( run_k2k_edges[i] );
		k2k_edge_unknowns[i] = 
#ifdef SRBA_WORKAROUND_MSVC9_DEQUE_BUG
			 rba_state.k2k_edges[run_k2k_edges[i]].pointer();
#else
			& rba_state.k2k_edges[run_k2k_edges[i]];
#endif
	}

	// k2f edges:
	std::vector<typename TSparseBlocksJacobians_dh_df::col_t*>  dh_df(nUnknowns_k2f);
	std::vector<TRelativeLandmarkPos*> k2f_edge_unknowns(nUnknowns_k2f);
	for (size_t i=0;i<nUnknowns_k2f;i++)
	{
		const TLandmarkID feat_id = run_feat_ids[i];
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

	// Mapping betwwen obs. indices in the residuals vector & the global list of all the observations:
	std::map<size_t,size_t>   obs_global_idx2residual_idx;

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

				involved_obs.push_back( TObsUsed (global_obs_idx, 
#ifdef SRBA_WORKAROUND_MSVC9_DEQUE_BUG
				&(*rba_state.all_observations[global_obs_idx])
#else
				&rba_state.all_observations[global_obs_idx]
#endif
				 ) );
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

					involved_obs.push_back( TObsUsed( global_obs_idx,  
#ifdef SRBA_WORKAROUND_MSVC9_DEQUE_BUG
						&(*rba_state.all_observations[global_obs_idx]) 
#else
						&rba_state.all_observations[global_obs_idx] 
#endif
					) );
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

			obs_global_idx2residual_idx[global_obs_idx] = obs_idx;

			involved_obs.push_back(TObsUsed(global_obs_idx, 
#ifdef SRBA_WORKAROUND_MSVC9_DEQUE_BUG
				&(*rba_state.all_observations[global_obs_idx])
#else
				&rba_state.all_observations[global_obs_idx] 
#endif
			) );
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

	VERBOSE_LEVEL(2) << "[OPT] KF roots whose spantrees need numeric-updates: " << mrpt::system::sprintf_container("%u", kfs_num_spantrees_to_update) <<std::endl;


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

#if 0  // Save a sparse block representation of the Jacobian.
	{
		static unsigned int dbg_idx = 0;
	 mrpt::math::CMatrixDouble Jbin;
		rba_state.lin_system.dh_dAp.getBinaryBlocksRepresentation(Jbin);
		Jbin.saveToTextFile(mrpt::format("sparse_jacobs_blocks_%05u.txt",dbg_idx), mrpt::math::MATRIX_FORMAT_INT );
		rba_state.lin_system.dh_dAp.saveToTextFileAsDense(mrpt::format("sparse_jacobs_%05u.txt",dbg_idx));
		++dbg_idx;
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

	if (nInvalidJacobs) {
		mrpt::system::setConsoleColor(mrpt::system::CONCOL_RED);
		VERBOSE_LEVEL(1) << "[OPT] " << nInvalidJacobs << " Jacobian blocks ignored for 'invalid'.\n";
		mrpt::system::setConsoleColor(mrpt::system::CONCOL_NORMAL);
	}

	VERBOSE_LEVEL(2) << "[OPT] Individual Jacobs: " << count_jacobians << " #k2k_edges=" << nUnknowns_k2k << " #k2f_edges=" << nUnknowns_k2f << " #obs=" << nObs << std::endl;
	VERBOSE_LEVEL(2) << "[OPT] k2k_edges to optimize: " << mrpt::system::sprintf_container("% u",run_k2k_edges) << std::endl;
	VERBOSE_LEVEL(2) << "[OPT] k2f_edges to optimize: " << mrpt::system::sprintf_container("% u",run_feat_ids) << std::endl;
	// Extra verbose: display initial value of each optimized pose:
	if (m_verbose_level>=2 && !run_k2k_edges.empty())
	{
		std::cout << "[OPT] k2k_edges to optimize, initial value(s):\n";
		ASSERT_(k2k_edge_unknowns.size()==run_k2k_edges.size())
		for (size_t i=0;i<run_k2k_edges.size();i++)
			std::cout << " k2k_edge: " <<k2k_edge_unknowns[i]->from << "=>" << k2k_edge_unknowns[i]->to << ",inv_pose=" << k2k_edge_unknowns[i]->inv_pose << std::endl;
	}

	// VERY IMPORTANT: For J^t*J to be invertible, we need a full rank Hessian:
	//    nObs*OBS_DIMS >= nUnknowns_k2k*POSE_DIMS+nUnknowns_k2f*LM_DIMS
	//
	ASSERT_ABOVEEQ_(OBS_DIMS*nObs,POSE_DIMS*nUnknowns_k2k+LM_DIMS*nUnknowns_k2f)

	// ----------------------------------------------------------------
	//         Iterative Levenberg Marquardt (LM) algorithm
	// ----------------------------------------------------------------
	// LevMar parameters:
	double nu = 2;
	const double max_gradient_to_stop = 1e-15;  // Stop if the infinity norm of the gradient is smaller than this
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

	double RMSE = std::sqrt(total_proj_error/nObs);

	out_info.num_observations     = nObs;
	out_info.num_jacobians        = count_jacobians;
	out_info.num_kf2kf_edges_optimized = run_k2k_edges.size();
	out_info.num_kf2lm_edges_optimized = run_feat_ids.size();
	out_info.num_total_scalar_optimized = nUnknowns_scalars;
	out_info.num_span_tree_numeric_updates = count_span_tree_num_update;
	out_info.total_sqr_error_init = total_proj_error;


	VERBOSE_LEVEL(1) << "[OPT] LM: Initial RMSE=" <<  RMSE << " #Jcbs=" << count_jacobians << " #k2k_edges=" << nUnknowns_k2k << " #k2f_edges=" << nUnknowns_k2f << " #obs=" << nObs << std::endl;

	if (parameters.srba.feedback_user_iteration)
		(*parameters.srba.feedback_user_iteration)(0,total_proj_error,RMSE);

	// Compute the gradient: "grad = J^t * (h(x)-z)"
	// ---------------------------------------------------------------------------------
	Eigen::VectorXd  minus_grad; // The negative of the gradient.

	DETAILED_PROFILING_ENTER("opt.compute_minus_gradient")
	compute_minus_gradient(/* Out: */ minus_grad, /* In: */ dh_dAp, dh_df, residuals, obs_global_idx2residual_idx);
	DETAILED_PROFILING_LEAVE("opt.compute_minus_gradient")


	// Build symbolic structures for Schur complement:
	// ---------------------------------------------------------------------------------
	my_solver_t my_solver(
		m_verbose_level, m_profiler, 
		HAp,Hf,HApf, // The different symbolic/numeric Hessian
		minus_grad,  // minus gradient of the Ap part
		nUnknowns_k2k,
		nUnknowns_k2f);
	// Notice: At this point, the constructor of "my_solver_t" might have already built the Schur-complement 
	// of HAp-HApf into HAp: it's overwritten there (Only if RBA_OPTIONS::solver_t::USE_SCHUR=true).

	const double MAX_LAMBDA = this->parameters.srba.max_lambda;

	// These are defined here to avoid allocatin/deallocating memory with each iteration:
#ifdef SRBA_WORKAROUND_MSVC9_DEQUE_BUG
	vector<stlplus::smart_ptr<k2k_edge_t> >            old_k2k_edge_unknowns;
	vector<stlplus::smart_ptr<pose_flag_t> >      old_span_tree; // In the same order than "list_of_required_num_poses"
	vector<stlplus::smart_ptr<TRelativeLandmarkPos> >  old_k2f_edge_unknowns;
#else
	vector<k2k_edge_t>            old_k2k_edge_unknowns;
	vector<pose_flag_t>      old_span_tree; // In the same order than "list_of_required_num_poses"
	vector<TRelativeLandmarkPos>  old_k2f_edge_unknowns;
#endif

#if SRBA_DETAILED_TIME_PROFILING
	const std::string sLabelProfilerLM_iter = mrpt::format("opt.lm_iteration_k2k=%03u_k2f=%03u", static_cast<unsigned int>(nUnknowns_k2k), static_cast<unsigned int>(nUnknowns_k2f) );
#endif

	// LevMar iterations -------------------------------------
	size_t iter; // Declared here so we can read the final # of iterations out of the "for" loop.
	bool   stop = false;
	for (iter=0; iter<this->parameters.srba.max_iters && !stop; iter++)
	{
		DETAILED_PROFILING_ENTER(sLabelProfilerLM_iter.c_str())

		// Try with different rho's until a better solution is found:
		double rho = 0;
		if (lambda>=MAX_LAMBDA)
		{
			stop=true;
			VERBOSE_LEVEL(2) << "[OPT] LM end criterion: lambda too large. " << lambda << ">=" <<MAX_LAMBDA<<endl;
		}
		if (RMSE < this->parameters.srba.max_error_per_obs_to_stop)
		{
			stop=true;
			VERBOSE_LEVEL(2) << "[OPT] LM end criterion: error too small. " << RMSE << "<" <<this->parameters.srba.max_error_per_obs_to_stop<<endl;
		}

		while(rho<=0 && !stop)
		{
			// -------------------------------------------------------------------------
			//  Build the matrix (Hessian+ \lambda I) and decompose it with Cholesky:
			// -------------------------------------------------------------------------
			if ( !my_solver.solve(lambda) )
			{
				// not positive definite so increase lambda and try again
				lambda *= nu;
				nu *= 2.;
				stop = (lambda>MAX_LAMBDA);

				VERBOSE_LEVEL(2) << "[OPT] LM iter #"<< iter << " NotDefPos in Cholesky. Retrying with lambda=" << lambda << std::endl;
				continue;
			}

			// Make a copy of the old edge values, just in case we need to restore them back...
			// ----------------------------------------------------------------------------------
			DETAILED_PROFILING_ENTER("opt.make_backup_copy_edges")

			old_k2k_edge_unknowns.resize(nUnknowns_k2k);
			for (size_t i=0;i<nUnknowns_k2k;i++)
			{
#ifdef SRBA_WORKAROUND_MSVC9_DEQUE_BUG
				old_k2k_edge_unknowns[i] = stlplus::smart_ptr<k2k_edge_t>( new k2k_edge_t(*k2k_edge_unknowns[i] ) );
#else
				old_k2k_edge_unknowns[i] = *k2k_edge_unknowns[i];
#endif
			}

			old_k2f_edge_unknowns.resize(nUnknowns_k2f);
			for (size_t i=0;i<nUnknowns_k2f;i++)
			{
#ifdef SRBA_WORKAROUND_MSVC9_DEQUE_BUG
				old_k2f_edge_unknowns[i] = stlplus::smart_ptr<TRelativeLandmarkPos>(new TRelativeLandmarkPos(*k2f_edge_unknowns[i]));
#else
				old_k2f_edge_unknowns[i] = *k2f_edge_unknowns[i];
#endif
			}

			DETAILED_PROFILING_LEAVE("opt.make_backup_copy_edges")

			// Add SE(2/3) deltas to the k2k edges:
			// ------------------------------------
			DETAILED_PROFILING_ENTER("opt.add_se3_deltas_to_frames")
			for (size_t i=0;i<nUnknowns_k2k;i++)
			{
				// edges_to_optimize:
				const pose_t &old_pose = k2k_edge_unknowns[i]->inv_pose;
				pose_t new_pose(mrpt::poses::UNINITIALIZED_POSE);

				// Use the Lie Algebra methods for the increment:
				const mrpt::math::CArrayDouble<POSE_DIMS> incr( & my_solver.delta_eps[POSE_DIMS*i] );
				pose_t  incrPose(mrpt::poses::UNINITIALIZED_POSE);
				se_traits_t::pseudo_exp(incr,incrPose);   // incrPose = exp(incr) (Lie algebra pseudo-exponential map)

				//new_pose =  old_pose  [+] delta
				//         = exp(delta) (+) old_pose
				new_pose.composeFrom(incrPose, old_pose);

				VERBOSE_LEVEL(3) << "[OPT] Update k2k_edge[" <<i<< "]: eps=" << incr.transpose() << "\n" << " such that: " << old_pose << " => " << new_pose << "\n";

				//  Overwrite problem graph:
				k2k_edge_unknowns[i]->inv_pose = new_pose;
			}
			DETAILED_PROFILING_LEAVE("opt.add_se3_deltas_to_frames")


			// Add R^3 deltas to the k2f edges:
			// ------------------------------------
			DETAILED_PROFILING_ENTER("opt.add_deltas_to_feats")
			for (size_t i=0;i<nUnknowns_k2f;i++)
			{
				const double *delta_feat = &my_solver.delta_eps[idx_start_f+LM_DIMS*i];
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
				for (size_t i=0;i<nReqNumPoses;i++) 
				{
#ifdef SRBA_WORKAROUND_MSVC9_DEQUE_BUG
					old_span_tree[i] = stlplus::smart_ptr<pose_flag_t>(new pose_flag_t);
					old_span_tree[i]->pose = list_of_required_num_poses[i]->pose;
#else
					old_span_tree[i].pose = list_of_required_num_poses[i]->pose;
#endif
				}
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

			const double new_RMSE = std::sqrt(new_total_proj_error/nObs);

			const double error_reduction_ratio = total_proj_error>0 ? (total_proj_error - new_total_proj_error)/total_proj_error : 0;

			// is this better or worse?
			// -----------------------------
			rho = (total_proj_error - new_total_proj_error)/ (my_solver.delta_eps.array()*(lambda*my_solver.delta_eps + minus_grad).array() ).sum();

			if(rho>0)
			{
				// Good: Accept new values

				// Recalculate Jacobians?
				bool do_relinearize = (error_reduction_ratio<0 || error_reduction_ratio> this->parameters.srba.min_error_reduction_ratio_to_relinearize );

				VERBOSE_LEVEL(2) << "[OPT] LM iter #"<< iter << " RMSE: " << RMSE << " -> " << new_RMSE <<  ", rho=" << rho << ", Err.reduc.ratio="<< error_reduction_ratio << " => Relinearize?:" << (do_relinearize ? "YES":"NO") << std::endl;
				if (parameters.srba.feedback_user_iteration)
					(*parameters.srba.feedback_user_iteration)(iter,new_total_proj_error,new_RMSE);

				// Switch variables to the temptative ones, which are now accepted:
				//  (swap where possible, since it's faster)
				// ---------------------------------------------------------------------
				residuals.swap( new_residuals );

				total_proj_error = new_total_proj_error;
				RMSE = new_RMSE;

				if (do_relinearize)
				{
					DETAILED_PROFILING_ENTER("opt.reset_Jacobs_validity")
					// Before evaluating Jacobians we must reset as "valid" all the involved observations.
					//  If needed, they'll be marked as invalid by the Jacobian evaluator if just one of the components
					//  for one observation leads to an error.
					for (size_t i=0;i<nObs;i++)
						rba_state.all_observations_Jacob_validity[ involved_obs[i].obs_idx ] = 1;

					DETAILED_PROFILING_LEAVE("opt.reset_Jacobs_validity")

					DETAILED_PROFILING_ENTER("opt.recompute_all_Jacobians")
					recompute_all_Jacobians(dh_dAp, dh_df);
					DETAILED_PROFILING_LEAVE("opt.recompute_all_Jacobians")

					// Recalculate Hessian:
					DETAILED_PROFILING_ENTER("opt.sparse_hessian_update_numeric")
					sparse_hessian_update_numeric(HAp);
					sparse_hessian_update_numeric(Hf);
					sparse_hessian_update_numeric(HApf);
					DETAILED_PROFILING_LEAVE("opt.sparse_hessian_update_numeric")

					my_solver.realize_relinearized();
				}

				// Update gradient:
				DETAILED_PROFILING_ENTER("opt.compute_minus_gradient")
				compute_minus_gradient(/* Out: */ minus_grad, /* In: */ dh_dAp, dh_df, residuals, obs_global_idx2residual_idx);
				DETAILED_PROFILING_LEAVE("opt.compute_minus_gradient")

				const double norm_inf_min_grad = mrpt::math::norm_inf(minus_grad);
				if (norm_inf_min_grad<=max_gradient_to_stop)
				{
					VERBOSE_LEVEL(2) << "[OPT] LM end criterion: norm_inf(minus_grad) below threshold: " << norm_inf_min_grad << " <= " <<max_gradient_to_stop<<endl;
					stop = true;
				}
				if (RMSE<this->parameters.srba.max_error_per_obs_to_stop)
				{
					VERBOSE_LEVEL(2) << "[OPT] LM end criterion: RMSE below threshold: " << RMSE << " < " <<this->parameters.srba.max_error_per_obs_to_stop<<endl;
					stop = true;
				}
				if (rho>this->parameters.srba.max_rho)
				{
					VERBOSE_LEVEL(2) << "[OPT] LM end criterion: rho above threshold: " << rho << " > " <<this->parameters.srba.max_rho<<endl;
					stop = true;
				}
				// Reset other vars:
				lambda *= 1.0/3.0; //std::max(1.0/3.0, 1-std::pow(2*rho-1,3.0) );
				nu = 2.0;

				my_solver.realize_lambda_changed();
			}
			else
			{
				DETAILED_PROFILING_ENTER("opt.failedstep_restore_backup")

				// Restore old values and retry again with a different lambda:
				//DON'T: rba_state.spanning_tree.num = old_span_tree; // NO! Don't do this, since existing pointers will break -> Copy elements one by one:
				{
					const size_t nReqNumPoses = list_of_required_num_poses.size();
					for (size_t i=0;i<nReqNumPoses;i++) 
					{
#ifdef SRBA_WORKAROUND_MSVC9_DEQUE_BUG
						const_cast<pose_flag_t*>(list_of_required_num_poses[i])->pose = old_span_tree[i]->pose;
#else
						const_cast<pose_flag_t*>(list_of_required_num_poses[i])->pose = old_span_tree[i].pose;
#endif
					}
				}

				// Restore old edge values:
				for (size_t i=0;i<nUnknowns_k2k;i++)
				{
					*k2k_edge_unknowns[i] =
#ifdef SRBA_WORKAROUND_MSVC9_DEQUE_BUG
					*old_k2k_edge_unknowns[i];
#else
					old_k2k_edge_unknowns[i];
#endif
				}
				for (size_t i=0;i<nUnknowns_k2f;i++)
				{
					*k2f_edge_unknowns[i] = 
#ifdef SRBA_WORKAROUND_MSVC9_DEQUE_BUG
					*old_k2f_edge_unknowns[i];
#else
					old_k2f_edge_unknowns[i];
#endif
				}

				DETAILED_PROFILING_LEAVE("opt.failedstep_restore_backup")

				VERBOSE_LEVEL(2) << "[OPT] LM iter #"<< iter << " no update,errs: " << sqrt(total_proj_error/nObs) << " < " << sqrt(new_total_proj_error/nObs) << " lambda=" << lambda <<endl;
				lambda *= nu;
				nu *= 2.0;
				stop = (lambda>MAX_LAMBDA);

				my_solver.realize_lambda_changed();
			}

		}; // end while rho

		DETAILED_PROFILING_LEAVE(sLabelProfilerLM_iter.c_str())

	} // end for LM "iter"

#if 0
	std::cout << "residuals" << std::endl;
	for( size_t r = 0; r < residuals.size(); ++r ) 
	{
		std::cout << involved_obs[r].k2f->obs.obs.feat_id << ","
			 << residuals[r][0] << "," 
			 << residuals[r][1] << "," 
			 << residuals[r][2] << "," 
			 << residuals[r][3];

		const double totalres = residuals[r][0]*residuals[r][0]+
			residuals[r][1]*residuals[r][1]+
			residuals[r][2]*residuals[r][2]+
			residuals[r][3]*residuals[r][3];

		if( totalres > 20 )
			std::cout << " <-- spurious( " << totalres << ")";
		
		std::cout << std::endl;
	}
	cout << "done" << std::endl;
#endif 

	// Final output info:
	out_info.total_sqr_error_final = total_proj_error;

	// Recover information on covariances?
	// ----------------------------------------------
	DETAILED_PROFILING_ENTER("opt.cov_recovery")
	rba_state.unknown_lms_inf_matrices.clear();
	switch (parameters.srba.cov_recovery)
	{
		case crpNone:
			break;
		case crpLandmarksApprox:
		{
			for (size_t i=0;i<nUnknowns_k2f;i++)
			{
				if (!my_solver.was_ith_feature_invertible(i))
					continue;

				const typename hessian_traits_t::TSparseBlocksHessian_f::col_t & col_i = Hf.getCol(i);
				ASSERT_(col_i.rbegin()->first==i)  // Make sure the last block matrix is the diagonal term of the upper-triangular matrix.

				const typename hessian_traits_t::TSparseBlocksHessian_f::matrix_t & inf_mat_src = col_i.rbegin()->second.num;
				typename hessian_traits_t::TSparseBlocksHessian_f::matrix_t & inf_mat_dst = rba_state.unknown_lms_inf_matrices[ run_feat_ids[i] ];
				inf_mat_dst = inf_mat_src;
			}
		}
		break;
		default: 
			throw std::runtime_error("Unknown value found for 'parameters.srba.cov_recovery'");
	}
	DETAILED_PROFILING_LEAVE("opt.cov_recovery")

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

	// Fill in any other extra info from the solver:
	DETAILED_PROFILING_ENTER("opt.get_extra_results")
	my_solver.get_extra_results(out_info.extra_results);
	DETAILED_PROFILING_LEAVE("opt.get_extra_results")

	// Extra verbose: display final value of each optimized pose:
	if (m_verbose_level>=2 && !run_k2k_edges.empty())
	{
		std::cout << "[OPT] k2k_edges to optimize, final value(s):\n";
		ASSERT_(k2k_edge_unknowns.size()==run_k2k_edges.size())
		for (size_t i=0;i<run_k2k_edges.size();i++)
			std::cout << " k2k_edge: " <<k2k_edge_unknowns[i]->from << "=>" << k2k_edge_unknowns[i]->to << ",inv_pose=" << k2k_edge_unknowns[i]->inv_pose << std::endl;
	}

	// Save (quick swap) the list of unknowns to the output structure, 
	//  now that these vectors are not needed anymore:
	out_info.optimized_k2k_edge_indices.swap(run_k2k_edges);
	out_info.optimized_landmark_indices.swap(run_feat_ids);

	m_profiler.leave("opt");
	out_info.obs_rmse = RMSE;

	VERBOSE_LEVEL(1) << "[OPT] Final RMSE=" <<  RMSE << " #iters=" << iter << "\n";
}


} }  // end namespaces

