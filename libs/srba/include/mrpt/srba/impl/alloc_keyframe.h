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

/** Append an empty new keyframe to the data structures
  * \return The ID of the new KF.
  * \note Runs in O(1)
  */
template <class KF2KF_POSE_TYPE,class LM_TYPE,class OBS_TYPE,class RBA_OPTIONS>
TKeyFrameID RbaEngine<KF2KF_POSE_TYPE,LM_TYPE,OBS_TYPE,RBA_OPTIONS>::alloc_keyframe()
{
	// ==== Assign a free ID to the new KF   : O(1) ====
	const TKeyFrameID new_kf_id = rba_state.keyframes.size();

	// ==== Create new KF struct: insert at end of std::map<>  : O(1) ====
	rba_state.keyframes.push_back( keyframe_info() );
	//keyframe_info &kfi_new = rba_state.keyframes.back();
	return new_kf_id;
}


} } // end NS
