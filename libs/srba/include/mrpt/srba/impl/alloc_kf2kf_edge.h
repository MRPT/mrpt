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

// See header & papers for docs
template <class KF2KF_POSE_TYPE,class LM_TYPE,class OBS_TYPE,class RBA_OPTIONS>
size_t TRBA_Problem_state<KF2KF_POSE_TYPE,LM_TYPE,OBS_TYPE,RBA_OPTIONS>::alloc_kf2kf_edge(
	const TPairKeyFrameID &ids,
	const pose_t &init_inv_pose_val )
{
	// Create edge:
#ifdef SRBA_WORKAROUND_MSVC9_DEQUE_BUG
	k2k_edges.push_back( stlplus::smart_ptr<k2k_edge_t>(new k2k_edge_t));         // O(1)
	k2k_edge_t & new_edge = *(*k2k_edges.rbegin());
#else
	k2k_edges.push_back(k2k_edge_t());         // O(1)
	k2k_edge_t & new_edge = *k2k_edges.rbegin();
#endif

	new_edge.from = ids.first;
	new_edge.to   = ids.second;

	ASSERT_(new_edge.from!=new_edge.to)

	new_edge.inv_pose = init_inv_pose_val;

	new_edge.id = k2k_edges.size()-1; // For convenience, save index within the same structure.

#ifdef _DEBUG
	{
		// Security consistency check for user introducing duplicated edges:
		std::deque<k2k_edge_t*> &edges = keyframes[ids.first ].adjacent_k2k_edges;
		for (size_t i=0;i<edges.size();++i)
		{
			const k2k_edge_t &e = *edges[i];
			if ( (e.to==ids.first && e.from==ids.second) ||
				 (e.from==ids.first && e.to==ids.second) )
			{
				throw std::runtime_error( mrpt::format("[alloc_kf2kf_edge] ERROR: Edge already exists between %u -> %u",static_cast<unsigned int>(ids.first),static_cast<unsigned int>(ids.second) ) );
			}
		}
	}
#endif

	// Update adjacency lists:  O(1)
	keyframes[ids.first ].adjacent_k2k_edges.push_back(&new_edge);
	keyframes[ids.second].adjacent_k2k_edges.push_back(&new_edge);

	// Expand dh_dAp Jacobian to make room for a new column for this new edge:
	const size_t remapIdx = new_edge.id;
	//TSparseBlocksJacobians_dh_dAp::col_t & col =
	lin_system.dh_dAp.appendCol(remapIdx);         // O(1) with map_as_vector

	return new_edge.id;

} // end of alloc_kf2kf_edge

} } // end NS
