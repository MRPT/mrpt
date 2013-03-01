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
