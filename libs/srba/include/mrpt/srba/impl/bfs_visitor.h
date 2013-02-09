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
template <
	class KF_VISITOR,
	class FEAT_VISITOR,
	class K2K_EDGE_VISITOR,
	class K2F_EDGE_VISITOR
	>
void RBA_Problem<KF2KF_POSE_TYPE,LM_TYPE,OBS_TYPE,RBA_OPTIONS>::bfs_visitor(
	const TKeyFrameID  root_id,
	const topo_dist_t  max_distance,
	KF_VISITOR       & kf_visitor,
	FEAT_VISITOR     & feat_visitor,
	K2K_EDGE_VISITOR & k2k_edge_visitor,
	K2F_EDGE_VISITOR & k2f_edge_visitor ) const
{
	set<TKeyFrameID>   kf_visited;
	set<const k2k_edge_t*> k2k_visited;
	set<TLandmarkID>   lm_visited;
	set<const k2f_edge_t*> k2f_visited;

	queue<TKeyFrameID> pending;
	map<TKeyFrameID,topo_dist_t> distances;

	pending.push(root_id);
	kf_visited.insert(root_id);
	distances[root_id] = 0;

	while (!pending.empty())
	{
		const TKeyFrameID next_kf = pending.front();
		pending.pop();

		const topo_dist_t cur_dist = distances[next_kf];

		kf_visitor.visit_kf(next_kf,cur_dist);

		// Get all connections of this node:
		ASSERTDEB_(next_kf < rba_state.keyframes.size())
		const keyframe_info & kfi = rba_state.keyframes[next_kf];

		// Visit all KF2FEAT edges and features themselves:
		for (size_t i=0;i<kfi.adjacent_k2f_edges.size();i++)
		{
			const k2f_edge_t* ed = kfi.adjacent_k2f_edges[i];
			const TLandmarkID lm_ID = ed->obs.obs.feat_id;
			if (!lm_visited.count(lm_ID))
			{
				if (feat_visitor.visit_filter_feat(lm_ID,cur_dist) )
					feat_visitor.visit_feat(lm_ID,cur_dist);
				lm_visited.insert(lm_ID);
			}
			if (!k2f_visited.count(ed))
			{
				if (k2f_edge_visitor.visit_filter_k2f(next_kf,ed,cur_dist) )
					k2f_edge_visitor.visit_k2f(next_kf,ed,cur_dist);
				k2f_visited.insert(ed);
			}
		}

				
		// Don't explore nearby keyframes if we are already at the maximum distance from root.
		if (cur_dist>=max_distance) // At most, we should reach cur_dist==max_distance, but just in case use ">="
			continue;

		// Visit all KF2KF edges and queue nearby keyframes:
		for (size_t i=0;i<kfi.adjacent_k2k_edges.size();i++)
		{
			const k2k_edge_t* ed = kfi.adjacent_k2k_edges[i];
			const TKeyFrameID new_kf = getTheOtherFromPair2(next_kf, *ed);
			if (!kf_visited.count(new_kf))
			{
				if (kf_visitor.visit_filter_kf(new_kf,cur_dist) )
				{
					pending.push(new_kf);
					distances[new_kf]=cur_dist+1;
				}
				kf_visited.insert(new_kf);
			}
			if (!k2k_visited.count(ed))
			{
				if (k2k_edge_visitor.visit_filter_k2k(next_kf,new_kf,ed,cur_dist) )
					k2k_edge_visitor.visit_k2k(next_kf,new_kf,ed,cur_dist);
				k2k_visited.insert(ed);
			}
		}
	}

}


} } // end NS
