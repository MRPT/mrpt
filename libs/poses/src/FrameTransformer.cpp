/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "poses-precomp.h"  // Precompiled headers

#include <mrpt/poses/FrameTransformer.h>  // for FrameTransformer, FrameTran...
#include <string>  // for string
#include <mrpt/system/datetime.h>  // for TTimeStamp, INVALID_TIMESTAMP
#include <mrpt/core/exceptions.h>  // for ASSERTMSG_

using namespace mrpt::poses;

// ------- FrameTransformerInterface --------
template <int DIM>
FrameTransformerInterface<DIM>::FrameTransformerInterface() = default;
template <int DIM>
FrameTransformerInterface<DIM>::~FrameTransformerInterface() = default;

namespace mrpt
{
namespace poses
{
// Explicit instantations:
template class FrameTransformerInterface<2>;
template class FrameTransformerInterface<3>;
}  // namespace poses
}  // namespace mrpt

// ------- FrameTransformer --------
template <int DIM>
FrameTransformer<DIM>::FrameTransformer() = default;
template <int DIM>
FrameTransformer<DIM>::~FrameTransformer() = default;

template <int DIM>
void FrameTransformer<DIM>::sendTransform(
	const std::string& parent_frame, const std::string& child_frame,
	const typename base_t::pose_t& child_wrt_parent,
	const mrpt::system::TTimeStamp& timestamp)
{
	m_pose_edges_buffer[parent_frame][child_frame] =
		TF_TreeEdge(child_wrt_parent, timestamp);
}

// future work: allow graph traversal and do pose chain composition?
// We dont need to replicate the full functionality of ROS tf, though... just
// what we really need.

template <int DIM>
FrameLookUpStatus FrameTransformer<DIM>::lookupTransform(
	const std::string& target_frame, const std::string& source_frame,
	typename base_t::lightweight_pose_t& child_wrt_parent,
	const mrpt::system::TTimeStamp query_time, const double timeout_secs)
{
	ASSERTMSG_(
		timeout_secs == .0,
		"timeout_secs!=0: Blocking calls not supported yet!");
	ASSERTMSG_(
		query_time == INVALID_TIMESTAMP,
		"`query_time` different than 'latest' not supported yet!");

	const auto& it_src = m_pose_edges_buffer.find(source_frame);
	if (it_src == m_pose_edges_buffer.end())
	{
		return LKUP_UNKNOWN_FRAME;
	}

	const auto& it_dst = it_src->second.find(target_frame);
	if (it_dst == it_src->second.end())
	{
		return LKUP_NO_CONNECTIVITY;
	}

	const TF_TreeEdge& te = it_dst->second;
	child_wrt_parent = te.pose.asTPose();

	return LKUP_GOOD;
}

namespace mrpt
{
namespace poses
{
// Explicit instantations:
template class FrameTransformer<2>;
template class FrameTransformer<3>;
}  // namespace poses
}  // namespace mrpt
