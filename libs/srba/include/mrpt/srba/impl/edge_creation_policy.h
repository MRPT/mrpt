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

/** Determines and creates the new kf2fk edges given the set of new observations: */
template <class KF2KF_POSE_TYPE,class LM_TYPE,class OBS_TYPE,class RBA_OPTIONS>
void RbaEngine<KF2KF_POSE_TYPE,LM_TYPE,OBS_TYPE,RBA_OPTIONS>::edge_creation_policy(
	const TKeyFrameID               new_kf_id,
	const typename traits_t::new_kf_observations_t   & obs,
	std::vector<TNewEdgeInfo> &new_k2k_edge_ids )
{
	using namespace std;
	switch (parameters.srba.edge_creation_policy)
	{
	// -----------------------------------------------------------
	//  Policy: Linear graph
	// -----------------------------------------------------------
	case ecpLinearGraph:
		{
			// The simplest policy: Always add a single edge (n-1) => (n)
			const pose_t init_inv_pose;

			TNewEdgeInfo nei;
			nei.has_aprox_init_val = true; // In a linear graph it's a reasonable approx. to make each KF start at the last KF pose, which is what means a null pose init val.

			nei.id = this->create_kf2kf_edge(new_kf_id, TPairKeyFrameID( new_kf_id-1, new_kf_id), obs, init_inv_pose);

			new_k2k_edge_ids.push_back(nei);
		}
		break;

	// -----------------------------------------------------------
	//  Policy: Submapping-like policy as in ICRA-2013 paper
	// -----------------------------------------------------------
	case ecpICRA2013:
		{
			ASSERT_(new_kf_id>=1)

			const size_t MINIMUM_OBS_TO_LOOP_CLOSURE = parameters.srba.min_obs_to_loop_closure;
			const size_t SUBMAP_SIZE = parameters.srba.submap_size; // In # of KFs
			const TKeyFrameID cur_localmap_center = SUBMAP_SIZE*((new_kf_id-1)/SUBMAP_SIZE);

			// For normal KFs, just connect it to its local area central KF:
			if (0!=(new_kf_id)%SUBMAP_SIZE)
			{
				TNewEdgeInfo nei;
				nei.has_aprox_init_val = false; // Filled in below

				nei.id = this->create_kf2kf_edge(new_kf_id, TPairKeyFrameID( cur_localmap_center, new_kf_id), obs );

				if (0==((new_kf_id-1)%SUBMAP_SIZE))
				{
					// This is the first KF after a new center, so if we add an edge to it we must be very close:

					this->rba_state.k2k_edges[nei.id]
#ifdef SRBA_WORKAROUND_MSVC9_DEQUE_BUG
					->
#else
					.
#endif
					inv_pose = pose_t();
				}
				else
				if (nei.id>=1)
				{
					// Idea: the new KF should be close to the last one.
#ifdef SRBA_WORKAROUND_MSVC9_DEQUE_BUG
					this->rba_state.k2k_edges[nei.id]->inv_pose = this->rba_state.k2k_edges[nei.id-1]->inv_pose;
#else
					this->rba_state.k2k_edges[nei.id].inv_pose = this->rba_state.k2k_edges[nei.id-1].inv_pose;
#endif
				}

				new_k2k_edge_ids.push_back(nei);

				VERBOSE_LEVEL(2) << "[edge_creation_policy] Created edge #"<< nei.id << ": "<< cur_localmap_center<<"->"<< new_kf_id << endl;
			}
			else
			{
				// Only for "submap centers", so we force the edges from the rest to the center to be optimizable, since
				//  many obs from different areas become dependent on those edges

				// Go thru all observations and for those already-seen LMs, check the distance between their base KFs and (i_id):
				// Make a list of base KFs of my new observations, ordered in descending order by # of shared observations:
				base_sorted_lst_t         obs_for_each_base_sorted;
				make_ordered_list_base_kfs(obs, obs_for_each_base_sorted);

				// Make vote list for each central KF:
				map<TKeyFrameID,size_t>  obs_for_each_area;
				for (base_sorted_lst_t::const_iterator it=obs_for_each_base_sorted.begin();it!=obs_for_each_base_sorted.end();++it)
				{
					const size_t      num_obs_this_base = it->first;
					const TKeyFrameID base_id = it->second;

					const TKeyFrameID this_localmap_center = SUBMAP_SIZE*(base_id/SUBMAP_SIZE);

					obs_for_each_area[this_localmap_center] += num_obs_this_base;
				}

				// Sort by votes:
				base_sorted_lst_t   obs_for_each_area_sorted;
				for (map<TKeyFrameID,size_t>::const_iterator it=obs_for_each_area.begin();it!=obs_for_each_area.end();++it)
					obs_for_each_area_sorted.insert( make_pair(it->second,it->first) );

				// Go thru candidate areas:
				for (base_sorted_lst_t::const_iterator it=obs_for_each_area_sorted.begin();it!=obs_for_each_area_sorted.end();++it)
				{
					const size_t      num_obs_this_base = it->first;
					const TKeyFrameID central_kf_id = it->second;

					VERBOSE_LEVEL(2) << "[edge_creation_policy] Consider: area central kf#"<< central_kf_id << " with #obs:"<< num_obs_this_base << endl;

					// Create edges to all these central KFs if they're too far:

					// Find the distance between "central_kf_id" <=> "new_kf_id"
					const TKeyFrameID from_id = new_kf_id;
					const TKeyFrameID to_id   = central_kf_id;

					typename rba_problem_state_t::TSpanningTree::next_edge_maps_t::const_iterator it_from = rba_state.spanning_tree.sym.next_edge.find(from_id);

					topo_dist_t  found_distance = numeric_limits<topo_dist_t>::max();

					if (it_from != rba_state.spanning_tree.sym.next_edge.end())
					{
						const map<TKeyFrameID,TSpanTreeEntry> &from_Ds = it_from->second;
						map<TKeyFrameID,TSpanTreeEntry>::const_iterator it_to_dist = from_Ds.find(to_id);

						if (it_to_dist != from_Ds.end())
							found_distance = it_to_dist->second.distance;
					}
					else
					{
						// The new KF doesn't still have any edge created to it, that's why we didn't found any spanning tree for it.
						// Since this means that the KF is aisolated from the rest of the world, leave the topological distance to infinity.
					}

					if ( found_distance>=parameters.srba.max_optimize_depth)
					{
						if (num_obs_this_base>=MINIMUM_OBS_TO_LOOP_CLOSURE)
						{
							// The KF is TOO FAR: We will need to create an additional edge:
							TNewEdgeInfo nei;

							nei.id = this->create_kf2kf_edge(new_kf_id, TPairKeyFrameID( central_kf_id, new_kf_id), obs);
							nei.has_aprox_init_val = false; // Will need to estimate this one

#ifdef SRBA_WORKAROUND_MSVC9_DEQUE_BUG
							this->rba_state.k2k_edges[nei.id]->inv_pose = this->rba_state.k2k_edges[nei.id-1]->inv_pose;
#else
							this->rba_state.k2k_edges[nei.id].inv_pose = this->rba_state.k2k_edges[nei.id-1].inv_pose;
#endif
							new_k2k_edge_ids.push_back(nei);

							VERBOSE_LEVEL(2) << "[edge_creation_policy] Created extra edge #"<< nei.id << ": "<< central_kf_id <<"->"<<new_kf_id << " with #obs: "<< num_obs_this_base<< endl;
						}
						else
						{
							VERBOSE_LEVEL(1) << "[edge_creation_policy] Skipped extra edge " << central_kf_id <<"->"<<new_kf_id << " with #obs: "<< num_obs_this_base << " for too few shared obs!" << endl;
						}
					}
				}

				// At least we must create 1 edge!
				if (new_k2k_edge_ids.empty())
				{
					// Try linking to a "non-central" KF but at least having the min. # of desired shared observations:
					if (!obs_for_each_base_sorted.empty())
					{
						const size_t     most_connected_nObs  = obs_for_each_base_sorted.begin()->first;
						const TKeyFrameID most_connected_kf_id = obs_for_each_base_sorted.begin()->second;
						if (most_connected_nObs>=MINIMUM_OBS_TO_LOOP_CLOSURE)
						{
							TNewEdgeInfo nei;

							nei.id = this->create_kf2kf_edge(new_kf_id, TPairKeyFrameID( most_connected_kf_id, new_kf_id), obs);
							nei.has_aprox_init_val = false; // Will need to estimate this one
							new_k2k_edge_ids.push_back(nei);

							VERBOSE_LEVEL(0) << "[edge_creation_policy] Created edge of last resort #"<< nei.id << ": "<< most_connected_kf_id <<"->"<<new_kf_id << " with #obs: "<< most_connected_nObs<< endl;
						}
					}
				}

				// Recheck: if even with the last attempt we don't have any edge, it's bad:
				ASSERTMSG_(new_k2k_edge_ids.size()>=1, mrpt::format("Error for new KF#%u: no suitable linking KF found with a minimum of %u common observation: the node becomes isolated of the graph!", static_cast<unsigned int>(new_kf_id),static_cast<unsigned int>(MINIMUM_OBS_TO_LOOP_CLOSURE) ))

				// Debug:
				if (new_k2k_edge_ids.size()>1 && m_verbose_level>=1)
				{
					cout << "\n[edge_creation_policy] Loop closure detected for KF#"<< new_kf_id << ", edges: ";
					for (size_t j=0;j<new_k2k_edge_ids.size();j++)
					{
#ifdef SRBA_WORKAROUND_MSVC9_DEQUE_BUG
						cout << rba_state.k2k_edges[new_k2k_edge_ids[j].id]->from <<"->"<<rba_state.k2k_edges[new_k2k_edge_ids[j].id]->to<<", ";
#else
						cout << rba_state.k2k_edges[new_k2k_edge_ids[j].id].from <<"->"<<rba_state.k2k_edges[new_k2k_edge_ids[j].id].to<<", ";
#endif
					}
					cout << endl;
				}

			} // for each obs.

		}
		break;

	// -----------------------------------------------------------
	//  Policy: Star Graph
	//   KFs have "global coordinates" (wrt KF 0), features are
	//   relative to their base KFs.
	// -----------------------------------------------------------
	case ecpStarGraph:
		{
			TNewEdgeInfo nei;

			const bool this_is_first = this->rba_state.k2k_edges.empty();

			// Edge: #0 -> NEW_KF
			nei.id = this->create_kf2kf_edge(new_kf_id, TPairKeyFrameID(0, new_kf_id), obs);

			// Idea: the new KF should be close to the last one.
			if (!this_is_first) {
				nei.has_aprox_init_val = true; // Use pose of the last edge since the new one should be close.
#ifdef SRBA_WORKAROUND_MSVC9_DEQUE_BUG
				this->rba_state.k2k_edges[nei.id]->inv_pose = this->rba_state.k2k_edges[nei.id-1]->inv_pose;
#else
				this->rba_state.k2k_edges[nei.id].inv_pose = this->rba_state.k2k_edges[nei.id-1].inv_pose;
#endif
			}
			else {
				nei.has_aprox_init_val = false;
			}

			new_k2k_edge_ids.push_back(nei);
		}
		break;

	default:
		THROW_EXCEPTION("Unknown value for 'parameters.edge_creation_policy'!")
	};

} // end of RbaEngine::determine_kf2kf_edges_to_create


/** (Aux method) Make a list of base KFs of my new observations, ordered in descending order by # of shared observations: 
 * Cost: O(N_obs)
*/
template <class KF2KF_POSE_TYPE,class LM_TYPE,class OBS_TYPE,class RBA_OPTIONS>
void RbaEngine<KF2KF_POSE_TYPE,LM_TYPE,OBS_TYPE,RBA_OPTIONS>::make_ordered_list_base_kfs(
	const typename traits_t::new_kf_observations_t & obs,
	base_sorted_lst_t            & obs_for_each_base_sorted,
	std::map<TKeyFrameID,size_t>       *out_obs_for_each_base ) const
{
	using namespace std;
	// Make a first pass to make a sorted list of base KFs, ordered by # of observations so we prefer edges to
	// strongly connected base KFs:
	map<TKeyFrameID,size_t> obs_for_each_base;

	for (typename traits_t::new_kf_observations_t::const_iterator itObs=obs.begin();itObs!=obs.end();++itObs)
	{
		const TLandmarkID lm_id = itObs->obs.feat_id;
		if (lm_id>=rba_state.all_lms.size()) continue; // It's a new LM

		const typename landmark_traits<LM_TYPE>::TLandmarkEntry &lme = rba_state.all_lms[lm_id];
		if (!lme.rfp) continue; // It's a new LM.

		const TKeyFrameID base_id = lme.rfp->id_frame_base;
		obs_for_each_base[base_id]++; // vote for this.
	}

	// Sort map<> by values:
	for (map<TKeyFrameID,size_t>::const_iterator it=obs_for_each_base.begin();it!=obs_for_each_base.end();++it)
		obs_for_each_base_sorted.insert( make_pair(it->second,it->first) );

	if (out_obs_for_each_base)
		out_obs_for_each_base->swap(obs_for_each_base);
}


} }  // end namespaces
