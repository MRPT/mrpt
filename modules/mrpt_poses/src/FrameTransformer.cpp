/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

#include <mrpt/core/exceptions.h>  // for ASSERTMSG_
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/FrameTransformer.h>  // for FrameTransformer, FrameTran...
#include <mrpt/system/datetime.h>         // for TTimeStamp, INVALID_TIMESTAMP

#include <deque>
#include <map>
#include <string>  // for string
#include <utility>
#include <vector>

using namespace mrpt::poses;

// ------- FrameTransformerInterface --------
template <size_t DIM>
FrameTransformerInterface<DIM>::FrameTransformerInterface() = default;
template <size_t DIM>
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
template <size_t DIM>
FrameTransformer<DIM>::FrameTransformer() = default;
template <size_t DIM>
FrameTransformer<DIM>::~FrameTransformer() = default;

template <size_t DIM>
void FrameTransformer<DIM>::sendTransform(
    const std::string& parent_frame,
    const std::string& child_frame,
    const typename base_t::pose_t& child_wrt_parent,
    const mrpt::system::TTimeStamp& timestamp)
{
  m_pose_edges_buffer[parent_frame][child_frame] = TF_TreeEdge(child_wrt_parent, timestamp);
}

template <size_t DIM>
FrameLookUpStatus FrameTransformer<DIM>::lookupTransform(
    const std::string& target_frame,
    const std::string& source_frame,
    typename base_t::light_type& child_wrt_parent,
    const mrpt::system::TTimeStamp query_time,
    const double timeout_secs)
{
  using local_pose_t = typename base_t::pose_t;

  ASSERTMSG_(timeout_secs == .0, "timeout_secs!=0: Blocking calls not supported yet!");
  ASSERTMSG_(
      query_time == INVALID_TIMESTAMP, "`query_time` different than 'latest' not supported yet!");

  // Undirected graph of frames: each edge can be traversed from the parent
  // (with the stored pose) or from the child (with its inverse).
  std::map<std::string, std::vector<std::pair<std::string, local_pose_t>>> adjacency;
  for (const auto& [parent, children] : m_pose_edges_buffer)
  {
    for (const auto& [child, edge] : children)
    {
      adjacency[parent].emplace_back(child, edge.pose);
      adjacency[child].emplace_back(parent, -edge.pose);
    }
  }

  if (adjacency.count(source_frame) == 0 || adjacency.count(target_frame) == 0)
  {
    return LKUP_UNKNOWN_FRAME;
  }

  // Breadth-first search from the source frame, keeping the pose of each
  // reached frame with respect to the source frame:
  std::map<std::string, local_pose_t> pose_wrt_source;
  pose_wrt_source[source_frame] = local_pose_t();
  std::deque<std::string> pending = {source_frame};
  while (!pending.empty() && pose_wrt_source.count(target_frame) == 0)
  {
    const std::string cur = pending.front();
    pending.pop_front();
    const local_pose_t cur_pose = pose_wrt_source.at(cur);
    for (const auto& [next, next_wrt_cur] : adjacency.at(cur))
    {
      if (pose_wrt_source.count(next) != 0)
      {
        continue;
      }
      pose_wrt_source[next] = cur_pose + next_wrt_cur;
      pending.push_back(next);
    }
  }

  const auto it = pose_wrt_source.find(target_frame);
  if (it == pose_wrt_source.end())
  {
    return LKUP_NO_CONNECTIVITY;
  }

  child_wrt_parent = it->second.asTPose();
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
