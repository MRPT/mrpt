/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2015, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#pragma once

#include <queue>

namespace mrpt { namespace srba {

// This is used mainly for 3D rendering
template <class KF2KF_POSE_TYPE,class LM_TYPE,class OBS_TYPE,class RBA_OPTIONS>
void RbaEngine<KF2KF_POSE_TYPE,LM_TYPE,OBS_TYPE,RBA_OPTIONS>::create_complete_spanning_tree(
	const TKeyFrameID   root_id,
	frameid2pose_map_t & span_tree,
	const size_t        max_depth,
	vector<bool>  * aux_ws
	) const
{
	MRPT_UNUSED_PARAM(aux_ws);
	span_tree.clear();

	set<TKeyFrameID>   visited;
	queue<TKeyFrameID> pending;
	map<TKeyFrameID,TBFSEntryEdges>    preceding;

	// ----------------------------------------------------------------------------------------------------------------
	// (1) Do a BFS to build a spanning tree, storing the shortest path and its depth for each KF within max_depth:
	// ----------------------------------------------------------------------------------------------------------------
	// Insert:
	pending.push(root_id);
	visited.insert(root_id);
	preceding[root_id].dist = 0;

	while (!pending.empty())
	{
		const TKeyFrameID next_kf = pending.front();
		pending.pop();

		const topo_dist_t cur_dist = preceding[next_kf].dist;

		if (cur_dist>=max_depth)
			continue;

		// Get all connections of this node:
		ASSERTDEB_(next_kf < rba_state.keyframes.size())
		const keyframe_info & kfi = rba_state.keyframes[next_kf];

		for (size_t i=0;i<kfi.adjacent_k2k_edges.size();i++)
		{
			const k2k_edge_t* ed = kfi.adjacent_k2k_edges[i];
			const TKeyFrameID new_kf = getTheOtherFromPair2(next_kf, *ed);
			if (!visited.count(new_kf))
			{
				pending.push(new_kf);
				visited.insert(new_kf);

				TBFSEntryEdges & p = preceding[new_kf];

				if (p.dist>cur_dist+1)
				{
					p.dist = cur_dist+1;
					p.prev = next_kf;
					p.edge = ed;
				}
			}
		}
	}

	// ----------------------------------------------------------------------------------------------------------------
	// (2) Sort all KFs in range by hiearchy, i.e. by increasing depth:
	// ----------------------------------------------------------------------------------------------------------------
	multimap<topo_dist_t,pair<TKeyFrameID,TBFSEntryEdges> >  kfs_by_depth;

	for (typename map<TKeyFrameID,TBFSEntryEdges>::const_iterator it=preceding.begin();it!=preceding.end();++it)
		kfs_by_depth.insert( make_pair( it->second.dist, *it ) );


	// ----------------------------------------------------------------------------------------------------------------
	// (3) Construct the pose of each KF by composing the poses along the tree, from the root to the leaves:
	// ----------------------------------------------------------------------------------------------------------------
	for (typename multimap<topo_dist_t,pair<TKeyFrameID,TBFSEntryEdges> >::const_iterator it=kfs_by_depth.begin();it!=kfs_by_depth.end();++it)
	{
		const topo_dist_t  kf_depth = it->first;
		const TKeyFrameID  kf_id    = it->second.first;
		const TBFSEntryEdges  & bfs_data = it->second.second;


		if (kf_depth==0)
		{
			// Root:
			span_tree[ kf_id ].pose = pose_t(); // Default: origin.
		}
		else
		{
			// All leaves:
			ASSERT_(bfs_data.edge!=NULL)

			// The pose of my parent KF:
			const pose_t & parent_pose = span_tree[ bfs_data.prev ].pose;

			// A ref to the placeholder for my pose:
			pose_t & my_pose = span_tree[ kf_id ].pose;

			// Is the edge direct or inverted?
			if (bfs_data.edge->to==kf_id)
			{
				// Edge: parent -> me
				//  "inv_pose" in edge is really the inverse pose of me w.r.t. my parent:
				my_pose.composeFrom(parent_pose,  -bfs_data.edge->inv_pose );
			}
			else
			{
				// Edge: me -> parent
				//  "inv_pose" in edge is directly the pose of me w.r.t. my parent:
				my_pose.composeFrom(parent_pose, bfs_data.edge->inv_pose );
			}
		}
	}

}


} } // end NS
