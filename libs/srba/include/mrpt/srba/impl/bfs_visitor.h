/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
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
void RbaEngine<KF2KF_POSE_TYPE,LM_TYPE,OBS_TYPE,RBA_OPTIONS>::bfs_visitor(
	const TKeyFrameID  root_id,
	const topo_dist_t  max_distance,
	const bool         rely_on_prebuilt_spanning_trees,
	KF_VISITOR       & kf_visitor,
	FEAT_VISITOR     & feat_visitor,
	K2K_EDGE_VISITOR & k2k_edge_visitor,
	K2F_EDGE_VISITOR & k2f_edge_visitor ) const
{
	set<TLandmarkID>   lm_visited;
	set<const k2k_edge_t*> k2k_visited;
	set<const k2f_edge_t*> k2f_visited;

	if (!rely_on_prebuilt_spanning_trees)
	{	// Don't use prebuilt spanning-trees

		set<TKeyFrameID>   kf_visited;
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
		} // end for each "pending"
	}
	else
	{	// Use prebuilt spanning-trees
		const typename rba_problem_state_t::TSpanningTree::TSpanningTreeSym & st_sym = rba_state.spanning_tree.sym;

		typename rba_problem_state_t::TSpanningTree::next_edge_maps_t::const_iterator it_ste = st_sym.next_edge.find(root_id);
		if (it_ste == st_sym.next_edge.end())
			return; // It might be that this is the first node in the graph/subgraph...

		const std::map<TKeyFrameID,TSpanTreeEntry> & root_ST = it_ste->second;

		// make a list with all the KFs in the root's ST, + the root itself:
		std::vector< std::pair<TKeyFrameID,topo_dist_t> >  KFs;
		KFs.reserve(root_ST.size()+1);

		KFs.push_back( std::pair<TKeyFrameID,topo_dist_t>(root_id, 0 /* distance */) );
		for (typename std::map<TKeyFrameID,TSpanTreeEntry>::const_iterator it=root_ST.begin();it!=root_ST.end();++it)
			KFs.push_back( std::pair<TKeyFrameID,topo_dist_t>(it->first,it->second.distance) );

		// Go thru the list:
		for (size_t i=0;i<KFs.size();i++)
		{
			const TKeyFrameID kf_id    = KFs[i].first;
			const size_t      cur_dist = KFs[i].second;

			// Don't explore nearby keyframes if we are already at the maximum distance from root.
			if (cur_dist>max_distance)
				continue;

			// Visit KF itself:
			if (kf_visitor.visit_filter_kf(kf_id,cur_dist) )
			{
				kf_visitor.visit_kf(kf_id, cur_dist);
			}

			// Get all connections of this KF:
			ASSERTDEB_(kf_id < rba_state.keyframes.size())
			const keyframe_info & kfi = rba_state.keyframes[kf_id];

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
					if (k2f_edge_visitor.visit_filter_k2f(kf_id,ed,cur_dist) )
						k2f_edge_visitor.visit_k2f(kf_id,ed,cur_dist);
					k2f_visited.insert(ed);
				}
			}

			// Visit all KF2KF edges:
			for (size_t i=0;i<kfi.adjacent_k2k_edges.size();i++)
			{
				const k2k_edge_t* ed = kfi.adjacent_k2k_edges[i];
				const TKeyFrameID new_kf = getTheOtherFromPair2(kf_id, *ed);

				if (!k2k_visited.count(ed))
				{
					if (k2k_edge_visitor.visit_filter_k2k(kf_id,new_kf,ed,cur_dist) )
						k2k_edge_visitor.visit_k2k(kf_id,new_kf,ed,cur_dist);
					k2k_visited.insert(ed);
				}
			}
		} // end for each KF node in the ST of root
	} // end if-else use STs
}


} } // end NS
