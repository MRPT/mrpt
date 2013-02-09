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


template <class KF2KF_POSE_TYPE,class LM_TYPE,class OBS_TYPE,class RBA_OPTIONS>
void RBA_Problem<KF2KF_POSE_TYPE,LM_TYPE,OBS_TYPE,RBA_OPTIONS>::optimize_local_area(
	const TKeyFrameID  root_id,
	const unsigned int win_size,
	TOptimizeExtraOutputInfo & out_info,
	const TOptimizeLocalAreaParams &params,
	const std::vector<size_t> & observation_indices_to_optimize
	)
{
	m_profiler.enter("optimize_local_area");

	// 1st) Find list of edges to optimize:
	// --------------------------------------------------
	m_profiler.enter("optimize_local_area.find_edges2opt");
	
	/** Aux visitor struct */
	struct VisitorOptimizeLocalArea
	{
		VisitorOptimizeLocalArea(const rba_problem_state_t & rba_state_, const TOptimizeLocalAreaParams &params_) : 
			rba_state(rba_state_),
			params(params_)
		{ }

		const rba_problem_state_t & rba_state;
		const TOptimizeLocalAreaParams &params;

		vector<size_t> k2k_edges_to_optimize, lm_IDs_to_optimize;
		map<TLandmarkID,size_t>  lm_times_seen;

		/* Implementation of FEAT_VISITOR */
		inline bool visit_filter_feat(const TLandmarkID lm_ID,const topo_dist_t cur_dist) 
		{
			return false; // Don't need to visit landmark nodes.
		}
		inline void visit_feat(const TLandmarkID lm_ID,const topo_dist_t cur_dist)
		{
			// Nothing to do
		}

		/* Implementation of KF_VISITOR */
		inline bool visit_filter_kf(const TKeyFrameID kf_ID,const topo_dist_t cur_dist) 
		{
			return (kf_ID<=params.max_visitable_kf_id);
		}
		inline void visit_kf(const TKeyFrameID kf_ID,const topo_dist_t cur_dist)
		{
			// Nothing to do.
		}

		/* Implementation of K2K_EDGE_VISITOR */
		inline bool visit_filter_k2k(const TKeyFrameID current_kf, const TKeyFrameID next_kf,const k2k_edge_t* edge, const topo_dist_t cur_dist) 
		{
			return true; // Visit all k2k edges
		}
		inline void visit_k2k(const TKeyFrameID current_kf, const TKeyFrameID next_kf,const k2k_edge_t* edge, const topo_dist_t cur_dist) 
		{
			if (params.optimize_k2k_edges)
				k2k_edges_to_optimize.push_back(edge->id);
		}

		/* Implementation of K2F_EDGE_VISITOR */
		inline bool visit_filter_k2f(const TKeyFrameID current_kf, const k2f_edge_t* edge, const topo_dist_t cur_dist) 
		{
			return params.optimize_landmarks; // Yes: visit all feature nodes if we're asked to
		}
		inline void visit_k2f(const TKeyFrameID current_kf, const k2f_edge_t* edge, const topo_dist_t cur_dist) 
		{
			if (!edge->feat_has_known_rel_pos)
			{
				const TLandmarkID lm_ID = edge->obs.obs.feat_id;
				if (++lm_times_seen[lm_ID] >= params.dont_optimize_landmarks_seen_less_than_n_times)
					lm_IDs_to_optimize.push_back(lm_ID);
			}
		}
	};

	VisitorOptimizeLocalArea my_visitor(this->rba_state,params);

	this->bfs_visitor(
		root_id,  // Starting keyframe
		win_size, // max. depth
		my_visitor, //kf_visitor,
		my_visitor, //feat_visitor,
		my_visitor, //k2k_edge_visitor,
		my_visitor  //k2f_edge_visitor 
		);
	
	m_profiler.leave("optimize_local_area.find_edges2opt");

	// 2nd) Optimize them:
	// -------------------------------
	if (!my_visitor.k2k_edges_to_optimize.empty() || !my_visitor.lm_IDs_to_optimize.empty())
	{
		this->optimize_edges(my_visitor.k2k_edges_to_optimize,my_visitor.lm_IDs_to_optimize, out_info, observation_indices_to_optimize);
	}

	m_profiler.leave("optimize_local_area");
}



} } // end NS
