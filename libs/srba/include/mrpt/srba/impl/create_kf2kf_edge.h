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

template <class KF2KF_POSE_TYPE,class LM_TYPE,class OBS_TYPE,class RBA_OPTIONS>
size_t RbaEngine<KF2KF_POSE_TYPE,LM_TYPE,OBS_TYPE,RBA_OPTIONS>::create_kf2kf_edge(
	const TKeyFrameID        new_kf_id,
	const TPairKeyFrameID  & new_edge,
	const typename traits_t::new_kf_observations_t   & obs,
	const pose_t &init_inv_pose_val )
{
	// 1) Create new kf2kf structures (all but stuff related to the spanning trees)
	// ---------------------------------------------------------------------------------
	const size_t ed_id = rba_state.alloc_kf2kf_edge( new_edge, init_inv_pose_val );     // O(1)

	m_profiler.enter("define_new_keyframe.st.update_symbolic");

	// 2) Update symbolic spanning trees:
	// ---------------------------------------------------------------------------------
	rba_state.spanning_tree.update_symbolic_new_node(
		new_kf_id,
		new_edge,
		parameters.srba.max_tree_depth,
		(parameters.srba.edge_creation_policy==ecpLinearGraph),   // check_all_obs_are_connected
		&obs   // Only needed under the "linear graph" policy, to determine missing parts of the spanning tree to be filled in here
		);

	m_profiler.leave("define_new_keyframe.st.update_symbolic");

	return ed_id;
}

} } // end NS
