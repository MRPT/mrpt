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

#define OBS_SUPER_VERBOSE   0

template <class KF2KF_POSE_TYPE,class LM_TYPE,class OBS_TYPE,class RBA_OPTIONS>
size_t RBA_Problem<KF2KF_POSE_TYPE,LM_TYPE,OBS_TYPE,RBA_OPTIONS>::add_observation(
	const TKeyFrameID            observing_kf_id,
	const typename observation_traits<OBS_TYPE>::observation_t     & new_obs,
	const array_landmark_t * fixed_relative_position,
	const array_landmark_t * unknown_relative_position_init_val
	)
{
	m_profiler.enter("add_observation");

	ASSERT_( !( fixed_relative_position!=NULL && unknown_relative_position_init_val!=NULL) ) // Both can't be !=NULL at once.

	const bool is_1st_time_seen = ( new_obs.feat_id>=rba_state.all_lms.size() || rba_state.all_lms[new_obs.feat_id].rfp==NULL );

	const bool is_fixed =
		// This is the first observation of a fixed landmark:
		(fixed_relative_position!=NULL)
		|| // or if it was observed before, get its feature type from stored structure:
		(!is_1st_time_seen && rba_state.all_lms[new_obs.feat_id].has_known_pos );

	// Append all new observation to raw vector:
	// ---------------------------------------------------------------------
	const size_t new_obs_idx = rba_state.all_observations.size();    // O(1)
	rba_state.all_observations.push_back(k2f_edge_t()); // Create new k2f_edge -- O(1)
	rba_state.all_observations_Jacob_validity.push_back(1);  // Also grow this vector (its content now are irrelevant, they'll be updated in optimization)
	char * const jacob_valid_bit = &(*rba_state.all_observations_Jacob_validity.rbegin());

	// Get a ref. to observation info, filled in below:
	k2f_edge_t & new_k2f_edge = *rba_state.all_observations.rbegin();

	// New landmark? Update LMs structures if this is the 1st time we see this landmark:
	// -----------------------------------------------------------------------
	if (is_1st_time_seen)
	{
		if (is_fixed)
		{	// LM with KNOWN relative position.
			// ----------------------------------
			TRelativeLandmarkPos new_rfp;
			new_rfp.id_frame_base = observing_kf_id;  // The current KF at creation time becomes the relative coordinate base.
			new_rfp.pos = *fixed_relative_position;

			// O(1) insertion if the obs.feat_id is the largest until now:
			typename TRelativeLandmarkPosMap::iterator it_new = rba_state.known_lms.insert(
				rba_state.known_lms.end(),
				typename TRelativeLandmarkPosMap::value_type(new_obs.feat_id, new_rfp )
				);

			// Add to list of all LMs:   Amortized O(1)
			if (new_obs.feat_id >= rba_state.all_lms.size()) rba_state.all_lms.resize(new_obs.feat_id+1);
			rba_state.all_lms[new_obs.feat_id] = typename landmark_traits<LM_TYPE>::TLandmarkEntry(true /*known pos.*/, &it_new->second);
		}
		else
		{	// LM with UNKNOWN relative position.
			// ----------------------------------
			// O(1) insertion if the feat_ID is the largest until now:

			TRelativeLandmarkPos new_rfp;
			new_rfp.id_frame_base = observing_kf_id;  // The current KF at creation time becomes the relative coordinate base.
			if (unknown_relative_position_init_val)
			     new_rfp.pos =  *unknown_relative_position_init_val;
			else {
				// Default landmark position: Invoke sensor's inverse model.
				sensor_model_t::inverse_sensor_model(new_rfp.pos,new_obs.obs_data,this->parameters.sensor);

				// Take into account the sensor pose wrt the KF:
				RBA_OPTIONS::sensor_pose_on_robot_t::template sensor2robot_point<LM_TYPE>(new_rfp.pos, this->parameters.sensor_pose );
			}

			typename TRelativeLandmarkPosMap::iterator it_new = rba_state.unknown_lms.insert(
				rba_state.unknown_lms.end(),
				typename TRelativeLandmarkPosMap::value_type( new_obs.feat_id, new_rfp )
				);

			// Add to list of all LMs:
			if (new_obs.feat_id >= rba_state.all_lms.size()) rba_state.all_lms.resize(new_obs.feat_id+1);
			rba_state.all_lms[new_obs.feat_id] = typename landmark_traits<LM_TYPE>::TLandmarkEntry(false /*unknown pos.*/, &it_new->second);

			// Expand Jacobian dh_df to accomodate a new column for a new unknown:
			// ("Remap indices" in dh_df for each column are the feature IDs of those feature with unknown positions)
			rba_state.lin_system.dh_df.appendCol( new_obs.feat_id );
		}
	}

	// Maintain a pointer to the relative position wrt its base keyframe:
	TRelativeLandmarkPos *lm_rel_pos = rba_state.all_lms[new_obs.feat_id].rfp;
	// and get the ID of the base keyframe for the observed landmark:
	const TKeyFrameID base_id = lm_rel_pos->id_frame_base;

	// Fill in kf-to-feature edge data:
	// ------------------------------------
	new_k2f_edge.obs.kf_id = observing_kf_id;
	new_k2f_edge.obs.obs   = new_obs;
	new_k2f_edge.obs.obs.obs_data.getAsArray(new_k2f_edge.obs.obs_arr);  // Save the observation data as an array now only once, and reuse it from now on in optimizations, etc.
	new_k2f_edge.is_first_obs_of_unknown = is_1st_time_seen && !is_fixed;
	new_k2f_edge.feat_has_known_rel_pos  = is_fixed;
	new_k2f_edge.feat_rel_pos = lm_rel_pos;


	// Update keyframe incident edge list:
	// ---------------------------------------------------------------------
	ASSERTDEB_( observing_kf_id < rba_state.keyframes.size() )

	keyframe_info &kf_info = rba_state.keyframes[ observing_kf_id ];   // O(1)

	// Incident edges:
	kf_info.adjacent_k2f_edges.push_back( &new_k2f_edge );   // Amortized O(1)


	// Update linear system:

	//  If the observed feat has a known rel. pos., only dh_dAp; otherwise, both dh_dAp and dh_df
	// ---------------------------------------------------------------------
	// Add a new (block) row for this new observation (row index = "new_obs_idx" defined above)
	// We must create a block for each edge in between the observing and the ref. base id.
	// Note: no error checking here in find's for efficiency...
	m_profiler.enter("add_observation.jacobs.sym");

	// ===========================
	// Jacob 1/2: dh_dAp
	// ===========================
	bool graph_says_ignore_this_obs = false;

	if (base_id!=new_k2f_edge.obs.kf_id) // If this is a feat with unknown rel.pos. and it's its first observation, the dh_dAp part of the Jacobian is empty.
	{
#if OBS_SUPER_VERBOSE
		std::cout << "Jacobian dh_dAp for obs #"<<new_obs_idx << " has these edges [obs_kf="<<observing_kf_id<< " base_kf="<< base_id<<"]:\n";
#endif

		// Since "all_edges" is symmetric, only the (i,j), i>j entries are stored:
		const TKeyFrameID from = std::max(observing_kf_id, base_id);
		const TKeyFrameID to   = std::min(observing_kf_id, base_id);

		const bool all_edges_inverse = (from!=observing_kf_id);

		typename rba_problem_state_t::TSpanningTree::all_edges_maps_t::const_iterator it_map = rba_state.spanning_tree.sym.all_edges.find(from);  // Was: observing
		ASSERTMSG_(it_map != rba_state.spanning_tree.sym.all_edges.end(), mrpt::format("No ST.all_edges found for observing_id=%u, base_id=%u", static_cast<unsigned int>(observing_kf_id), static_cast<unsigned int>(base_id) ) )

		typename std::map<TKeyFrameID, typename rba_problem_state_t::k2k_edge_vector_t >::const_iterator it_obs_ed = it_map->second.find(to);
		//ASSERTMSG_(it_obs_ed != it_map->second.end(), mrpt::format("No spanning-tree found from KF #%u to KF #%u, base of observation of landmark #%u", static_cast<unsigned int>(observing_kf_id),static_cast<unsigned int>(base_id),static_cast<unsigned int>(new_obs.feat_id) ))

		if (it_obs_ed != it_map->second.end())
		{
			const typename rba_problem_state_t::k2k_edge_vector_t & obs_edges = it_obs_ed->second;
			ASSERT_(!obs_edges.empty())

			TKeyFrameID curKF = observing_kf_id; // The backward running index towards "base_id"
			for (size_t j=0;j<obs_edges.size();j++)
			{
				const size_t i = all_edges_inverse ?  (obs_edges.size()-j-1) : j;

				ASSERT_(curKF!=to)

				const size_t edge_id = obs_edges[i]->id;

				// Normal direction: from -> to, so "to"=="curKF" (we go backwards)
				const bool normal_dir = (obs_edges[i]->to==curKF);

#if OBS_SUPER_VERBOSE
				std::cout << " * edge #"<<edge_id<< ": "<<obs_edges[i]->from <<" => "<<obs_edges[i]->to << " (inverse: " << (normal_dir ? "no":"yes") << ")\n";
#endif

				// Get sparse block column:
				typename TSparseBlocksJacobians_dh_dAp::col_t & col = rba_state.lin_system.dh_dAp.getCol(edge_id);

				// Create new entry: O(1) optimized insert if obs_idx is the largest index (as it will normally be):
				typename TSparseBlocksJacobians_dh_dAp::col_t::iterator it_new_entry = col.insert(
					col.end(),
					typename TSparseBlocksJacobians_dh_dAp::col_t::value_type( new_obs_idx, typename TSparseBlocksJacobians_dh_dAp::TEntry() )
					);

				typename TSparseBlocksJacobians_dh_dAp::TEntry & entry = it_new_entry->second;

				entry.sym.obs_idx          = new_obs_idx;
				entry.sym.is_valid         = jacob_valid_bit;
				entry.sym.edge_normal_dir  = normal_dir;
				entry.sym.kf_d             = curKF;
				entry.sym.kf_base          = base_id;
				entry.sym.feat_rel_pos     = lm_rel_pos;
				entry.sym.k2k_edge_id      = edge_id;
				//entry.sym.rel_poses_path   = &obs_edges;  // Path OBSERVING_KF -> BASE_KF

				// Pointers to placeholders of future numeric results of the spanning tree:
				entry.sym.rel_pose_base_from_d1 = & rba_state.spanning_tree.num[curKF][base_id];
				entry.sym.rel_pose_d1_from_obs  =
					(curKF==observing_kf_id) ?
						NULL // Use special value "NULL" when the CPose is fixed to the origin.
						:
						& rba_state.spanning_tree.num[observing_kf_id][curKF];

				// next node after this edge is:
				curKF = normal_dir ? obs_edges[i]->from : obs_edges[i]->to;
			}
		}
		else
		{
			// Ignore this obs...
			graph_says_ignore_this_obs=true;
		}
	}

	// ===========================
	// Jacob 2/2: dh_df
	// ===========================
	if (!is_fixed && !graph_says_ignore_this_obs) // Only for features with unknown rel.pos.
	{
		// "Remap indices" in dh_df for each column are the feature IDs of those feature with unknown positions.
		const size_t remapIdx = new_k2f_edge.obs.obs.feat_id;

		const mrpt::utils::map_as_vector<size_t,size_t> &dh_df_remap = rba_state.lin_system.dh_df.getColInverseRemappedIndices();
		const mrpt::utils::map_as_vector<size_t,size_t>::const_iterator it_idx = dh_df_remap.find(remapIdx);  // O(1) in mrpt::utils::map_as_vector()
		ASSERT_(it_idx!=dh_df_remap.end())

		const size_t col_idx = it_idx->second;

#if OBS_SUPER_VERBOSE
		cout << "dh_df: col_idx=" << col_idx << " feat_id=" << remapIdx << " obs_idx=" << new_obs_idx <<  endl;
#endif
		// Get sparse block column:
		typename TSparseBlocksJacobians_dh_df::col_t & col = rba_state.lin_system.dh_df.getCol(col_idx);

		// Create new entry: O(1) optimized insert if obs_idx is the largest index (as it will normally be):
		typename TSparseBlocksJacobians_dh_df::col_t::iterator it_new_entry = col.insert(
			col.end(),
			typename TSparseBlocksJacobians_dh_df::col_t::value_type( new_obs_idx, typename TSparseBlocksJacobians_dh_df::TEntry() )
			);

		typename TSparseBlocksJacobians_dh_df::TEntry & entry = it_new_entry->second;

		entry.sym.obs_idx   = new_obs_idx;
		entry.sym.is_valid  = jacob_valid_bit;

		// Pointer to relative position:
		entry.sym.feat_rel_pos = lm_rel_pos;

		// Pointers to placeholders of future numeric results of the spanning tree:
		entry.sym.rel_pose_base_from_obs  =
			(new_k2f_edge.is_first_obs_of_unknown) ?
				NULL // Use special value "NULL" when the CPose is fixed to the origin.
				:
				& rba_state.spanning_tree.num[observing_kf_id][base_id];
	}

	m_profiler.leave("add_observation.jacobs.sym");

	m_profiler.leave("add_observation");

	return new_obs_idx;
}

} } // end NS
