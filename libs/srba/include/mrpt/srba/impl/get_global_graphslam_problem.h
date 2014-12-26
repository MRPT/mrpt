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

/** Exports all the keyframes and landmarks as a directed graph in DOT (graphviz) format */
template <class KF2KF_POSE_TYPE,class LM_TYPE,class OBS_TYPE,class RBA_OPTIONS>
template <class POSE_GRAPH>
void RbaEngine<KF2KF_POSE_TYPE,LM_TYPE,OBS_TYPE,RBA_OPTIONS>::get_global_graphslam_problem(
	POSE_GRAPH &global_graph,
	const ExportGraphSLAM_Params &params
	) const
{
	global_graph.clear();
	if (rba_state.keyframes.empty()) 
		return; // Nothing to do
	
	// 1) Initialize global poses with a Spanning-tree:
	// ------------------------------------------------
	frameid2pose_map_t  spantree;
	create_complete_spanning_tree(params.root_kf_id,spantree); // Go thru COMPLETE graph (unbounded complexity, non O(1) )

	// For each key-frame, add a 3D corner:
	for (typename frameid2pose_map_t::const_iterator itP = spantree.begin();itP!=spantree.end();++itP)
	{
		const TKeyFrameID kf_id = itP->first;
		global_graph.nodes[kf_id] = itP->second.pose;
	}

	// 2) Save KF-to-KF relative poses as edges:
	// ------------------------------------------------
	for (typename k2k_edges_deque_t::const_iterator itEdge=rba_state.k2k_edges.begin();itEdge!=rba_state.k2k_edges.end();++itEdge)
	{
		const k2k_edge_t & edge = *itEdge;
		// Edges in RBA store *inverse* poses "from"->"to". 
		// Save as "normal" poses "to"->"from"		
		global_graph.insertEdgeAtEnd(edge.to, edge.from, edge.inv_pose);
	}

}


} }  // end namespaces
