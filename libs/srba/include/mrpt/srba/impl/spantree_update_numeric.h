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

#include <mrpt/math/CSparseMatrix.h>
#include <memory>  // for auto_ptr<>
#include <queue>

namespace mrpt { namespace srba {

using namespace std;

#define UPDATE_NUM_ST_VERBOSE  0
#define DEBUG_GARBAGE_FILL_ALL_NUMS	0

/** Updates all the numeric SE(3) poses from a given entry from \a sym.all_edges[i] */
template <class KF2KF_POSE_TYPE,class LM_TYPE,class OBS_TYPE>
size_t TRBA_Problem_state<KF2KF_POSE_TYPE,LM_TYPE,OBS_TYPE>::TSpanningTree::update_numeric_only_all_from_node(
	const typename all_edges_maps_t::const_iterator & it,
	bool skip_marked_as_uptodate)
{
	// num[SOURCE] |--> map[TARGET] = CPose3D of TARGET as seen from SOURCE
	const TKeyFrameID id_from = it->first;
	frameid2pose_map_t &frameid2pose_map = num[id_from];   // O(1) with map_as_vector

	for (typename std::map<TKeyFrameID, k2k_edge_vector_t >::const_iterator itE = it->second.begin();itE != it->second.end();++itE)
	{
		const TKeyFrameID id_to   = itE->first;

		pose_flag_t & i2j = frameid2pose_map[id_to];
		pose_flag_t & j2i = num[id_to][id_from];  // O(1) with map_as_vector

		if (skip_marked_as_uptodate && i2j.updated && j2i.updated)
			continue;

		// Go recompute this pose:
		const k2k_edge_vector_t &ev = itE->second;

		// Accumulate inverse poses in the order established by the path:
		mrpt::poses::CPose3D accum;
		TKeyFrameID curKF = id_from;

#if UPDATE_NUM_ST_VERBOSE
		std::cout << "ST.NUM["<<id_from<<"]["<<id_to<<"] : ";
#endif
		for (size_t k=0;k<ev.size();k++)
		{
			if(ev[k]->to==curKF)  // Inverse poses means we should face all arcs by the "head" (arrow) side
			{
				accum.composeFrom(accum, ev[k]->inv_pose );
				curKF = ev[k]->from;
#if UPDATE_NUM_ST_VERBOSE
				std::cout << "->"<<curKF;
#endif
			}
			else
			{
				accum.composeFrom(accum, -ev[k]->inv_pose );  // unary "-" operator inverts SE(3) poses
				curKF = ev[k]->to;
#if UPDATE_NUM_ST_VERBOSE
				std::cout << "<-"<<curKF;
#endif
			}
		}

		// Save in map:
		i2j.pose = accum;
		i2j.updated = true;

		// And also symmetric (inverse) pose:
		j2i.pose = -accum;
		j2i.updated = true;

#if UPDATE_NUM_ST_VERBOSE
		std::cout << " "<< accum.asString() << std::endl;
#endif
	}

	return it->second.size();
}

template <class KF2KF_POSE_TYPE,class LM_TYPE,class OBS_TYPE>
void setAllNumericToGarbage(typename TRBA_Problem_state<KF2KF_POSE_TYPE,LM_TYPE,OBS_TYPE>::TSpanningTree &st)
{
	// Mark all numeric values to trash so we detect if some goes un-initialized.
	mrpt::math::CMatrixDouble33 R_trash;
	for (int i=0;i<3;i++)
		for (int j=0;j<3;j++)
			R_trash(i,j) = std::numeric_limits<double>::quiet_NaN();

	for (typename kf2kf_pose_traits<KF2KF_POSE_TYPE>::TRelativePosesForEachTarget::iterator it=st.num.begin();it!=st.num.end();++it)
	{
		typename kf2kf_pose_traits<KF2KF_POSE_TYPE>::frameid2pose_map_t & m = it->second;
		for (typename kf2kf_pose_traits<KF2KF_POSE_TYPE>::frameid2pose_map_t::iterator it2=m.begin();it2!=m.end();++it2)
		{
			for (int i=0;i<3;i++)
				it2->second.pose.m_coords[i] = std::numeric_limits<double>::quiet_NaN();
			it2->second.pose.setRotationMatrix( R_trash );
		}
	}
}

template <class KF2KF_POSE_TYPE,class LM_TYPE,class OBS_TYPE>
size_t TRBA_Problem_state<KF2KF_POSE_TYPE,LM_TYPE,OBS_TYPE>::TSpanningTree::update_numeric(bool skip_marked_as_uptodate)
{
#if DEBUG_GARBAGE_FILL_ALL_NUMS
	setAllNumericToGarbage(*this);
#endif
	size_t pose_count = 0;
	for (typename all_edges_maps_t::const_iterator it=sym.all_edges.begin();it!=sym.all_edges.end();++it)
		pose_count += update_numeric_only_all_from_node(it,  skip_marked_as_uptodate);
	return pose_count;
}

template <class KF2KF_POSE_TYPE,class LM_TYPE,class OBS_TYPE>
size_t TRBA_Problem_state<KF2KF_POSE_TYPE,LM_TYPE,OBS_TYPE>::TSpanningTree::update_numeric(const std::set<TKeyFrameID> & kfs_to_update,bool skip_marked_as_uptodate)
{
#if DEBUG_GARBAGE_FILL_ALL_NUMS
	setAllNumericToGarbage(*this);
#endif

	size_t pose_count = 0;
	for (std::set<TKeyFrameID>::const_iterator it=kfs_to_update.begin();it!=kfs_to_update.end();++it)
	{
		typename all_edges_maps_t::const_iterator it_edge=sym.all_edges.find( *it );
		ASSERT_(it_edge!=sym.all_edges.end())

		pose_count += update_numeric_only_all_from_node(it_edge,  skip_marked_as_uptodate);
	}
	return pose_count;
}

} } // end NS
