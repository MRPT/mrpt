/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2015, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#pragma once

namespace mrpt { namespace srba {

// The main entry point of SRBA. See .h and papers for docs.
template <class KF2KF_POSE_TYPE,class LM_TYPE,class OBS_TYPE,class RBA_OPTIONS>
void RbaEngine<KF2KF_POSE_TYPE,LM_TYPE,OBS_TYPE,RBA_OPTIONS>::define_new_keyframe(
	const typename traits_t::new_kf_observations_t  & obs,
	TNewKeyFrameInfo  & out_new_kf_info,
	const bool          run_local_optimization )
{
	m_profiler.enter("define_new_keyframe");

	out_new_kf_info.clear();

	// Update KFs data structures ;  O(1)
	// ------------------------------------
	const TKeyFrameID new_kf_id = alloc_keyframe();

	// Apply edge-creation policy to decide how to handle loop closures, etc.
	// -----------------------------------------------------------------------------
	// ==== Determine what edges to create:  O(TBD) ====
	m_profiler.enter("define_new_keyframe.determine_edges");

	// Keep a list of the new kf2kf edges, whose initial values are indeterminate:
	std::vector<TNewEdgeInfo> new_k2k_edge_ids;
	determine_kf2kf_edges_to_create(new_kf_id,obs, new_k2k_edge_ids);   // ***** here's the beef! *****

	m_profiler.leave("define_new_keyframe.determine_edges");


	// Expand symbolic Jacobians to accomodate new observations:      O( No * (P+log C) )
	// -----------------------------------------------------------------------------
	m_profiler.enter("define_new_keyframe.add_observations");

	for (typename new_kf_observations_t::const_iterator it_obs = obs.begin();it_obs != obs.end();++it_obs)
	{
		const typename landmark_traits_t::array_landmark_t *fixed_rel_pos       = it_obs->is_fixed                 ? &it_obs->feat_rel_pos : NULL;
		const typename landmark_traits_t::array_landmark_t *unk_rel_pos_initval = it_obs->is_unknown_with_init_val ? &it_obs->feat_rel_pos : NULL;

		this->add_observation( new_kf_id, it_obs->obs, fixed_rel_pos, unk_rel_pos_initval );
	}

	m_profiler.leave("define_new_keyframe.add_observations");

	// Update SLAM estimation:
	// -----------------------------------------------------------------------------
	if (run_local_optimization)
	{
		// Try to initialize the new edges in separate optimizations?
		if (parameters.srba.optimize_new_edges_alone)
		{
			// Do it one by one so we can detect rank-deficient situations, etc.
			if (!new_k2k_edge_ids.empty())
			{
				m_profiler.enter("define_new_keyframe.opt_new_edges");

				// temporarily disable robust kernel for initialization (faster)
				const bool old_kernel = parameters.srba.use_robust_kernel;
				parameters.srba.use_robust_kernel= parameters.srba.use_robust_kernel_stage1;

				std::vector<size_t>  k2f_edges_to_opt;  // Empty: only initialize k2k edges.
				std::vector<size_t>  k2k_edges_to_opt(1);

				for (size_t i=0;i<new_k2k_edge_ids.size();i++)
				{
					if (new_k2k_edge_ids[i].has_aprox_init_val)
						continue;  // Already initialized, can skip it.
					k2k_edges_to_opt[0] = new_k2k_edge_ids[i].id ;

					//TOptimizeExtraOutputInfo  init_opt_info;
					this->optimize_edges(
						k2k_edges_to_opt,
						k2f_edges_to_opt,
						out_new_kf_info.optimize_results_stg1 /*init_opt_info*/
						);
				}

				parameters.srba.use_robust_kernel = old_kernel;

				m_profiler.leave("define_new_keyframe.opt_new_edges");
			}
		}

		m_profiler.enter("define_new_keyframe.optimize");

		TOptimizeLocalAreaParams opt_params; // Default values

		this->optimize_local_area(
			new_kf_id, // root node
			parameters.srba.max_optimize_depth,   // win size
			out_new_kf_info.optimize_results,
			opt_params
			);

		m_profiler.leave("define_new_keyframe.optimize");
	}


	// Fill out_new_kf_info
	// -----------------------------------------
	out_new_kf_info.kf_id = new_kf_id;
	out_new_kf_info.created_edge_ids.swap( new_k2k_edge_ids ); // swap, faster than copy

	m_profiler.leave("define_new_keyframe");

	VERBOSE_LEVEL(1) << "[define_new_keyframe] Done. New KF #" << out_new_kf_info.kf_id << " with " << out_new_kf_info.created_edge_ids.size() << " new edges.\n";
} // end of RbaEngine::define_new_keyframe


} } // end NS
