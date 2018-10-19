/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/system/datetime.h>
#include <mrpt/core/aligned_std_map.h>
#include <mrpt/poses/SE_traits.h>
#include <string>
#include <map>

namespace mrpt::poses
{
enum FrameLookUpStatus
{
	LKUP_GOOD = 0,
	LKUP_UNKNOWN_FRAME,
	LKUP_NO_CONNECTIVITY,
	LKUP_EXTRAPOLATION_ERROR
};

/** Virtual base class for interfaces to a [ROS
 * tf2](http://wiki.ros.org/tf2)-like
 * service capable of "publishing" and "looking-up" relative poses between two
 * "coordinate frames".
 * Use derived classes for:
 *  - wrapping real ROS tf (TO-DO in
 * [mrpt-bridge](http://wiki.ros.org/mrpt_bridge)), or
 *  - using a pure MRPT standalone TF service with mrpt::poses::FrameTransformer
 *
 * Frame IDs are strings.
 * MRPT modules use the standard ROS [REP
 * 105](http://www.ros.org/reps/rep-0105.html#coordinate-frames)
 * document regarding common names for frames:
 *  - `base_link`: "the robot"
 *  - `odom`: Origin for odometry
 *  - `map`: Origin for "the map"
 *
 * \tparam DIM Can be 2 for SE(2), 2D transformations; or 3 for SE(3), 3D
 * transformations.
 * \ingroup poses_grp
 * \sa FrameTransformer, CPose3D
 */
template <int DIM>
class FrameTransformerInterface
{
   public:
	/** This will be mapped to CPose2D (DIM=2) or CPose3D (DIM=3) */
	using pose_t = typename SE_traits<DIM>::pose_t;
	/** This will be mapped to mrpt::math::TPose2D (DIM=2) or
	 * mrpt::math::TPose3D (DIM=3) */
	using lightweight_pose_t = typename SE_traits<DIM>::lightweight_pose_t;

	FrameTransformerInterface();
	virtual ~FrameTransformerInterface();

	/** Publish a time-stampped transform between two frames */
	virtual void sendTransform(
		const std::string& parent_frame, const std::string& child_frame,
		const pose_t& child_wrt_parent,
		const mrpt::system::TTimeStamp& timestamp = mrpt::system::now()) = 0;

	/** Queries the current pose of `target_frame` wrt ("as seen from")
	 * `source_frame`.
	 * It tries to return the pose at the given timepoint, unless it is
	 * INVALID_TIMESTAMP (default),
	 * which means returning the latest know transformation.
	 */
	virtual FrameLookUpStatus lookupTransform(
		const std::string& target_frame, const std::string& source_frame,
		lightweight_pose_t& child_wrt_parent,
		const mrpt::system::TTimeStamp query_time = INVALID_TIMESTAMP,
		/** Timeout */
		const double timeout_secs = .0) = 0;

};  // End of class def.

/** See docs in FrameTransformerInterface.
 *   This class is an implementation for standalone (non ROS) applications.
 * \ingroup poses_grp
 * \sa FrameTransformerInterface
 */
template <int DIM>
class FrameTransformer : public FrameTransformerInterface<DIM>
{
   public:
	using base_t = FrameTransformerInterface<DIM>;

	FrameTransformer();
	~FrameTransformer() override;

	// See base docs
	void sendTransform(
		const std::string& parent_frame, const std::string& child_frame,
		const typename base_t::pose_t& child_wrt_parent,
		const mrpt::system::TTimeStamp& timestamp =
			mrpt::system::now()) override;
	// See base docs
	FrameLookUpStatus lookupTransform(
		const std::string& target_frame, const std::string& source_frame,
		typename base_t::lightweight_pose_t& child_wrt_parent,
		const mrpt::system::TTimeStamp query_time = INVALID_TIMESTAMP,
		const double timeout_secs = .0) override;

	/** \overload */
	FrameLookUpStatus lookupTransform(
		const std::string& target_frame, const std::string& source_frame,
		typename base_t::pose_t& child_wrt_parent,
		const mrpt::system::TTimeStamp query_time = INVALID_TIMESTAMP,
		/** Timeout */
		const double timeout_secs = .0)
	{
		typename base_t::lightweight_pose_t p;
		FrameLookUpStatus ret = lookupTransform(
			target_frame, source_frame, p, query_time, timeout_secs);
		child_wrt_parent = typename base_t::pose_t(p);
		return ret;
	}

   protected:
	// double m_max_extrapolation_time;  //!< for extrapolation in the past or
	// in the future [s]
	// double m_max_age_pose_cache;      //!< Max age of stored poses [s]

	struct TF_TreeEdge
	{
		// TODO: CPose{2,3}DInterpolator?
		typename base_t::pose_t pose;
		mrpt::system::TTimeStamp timestamp;

		TF_TreeEdge(
			const typename base_t::pose_t& pose_,
			const mrpt::system::TTimeStamp& timestamp_)
			: pose(pose_), timestamp(timestamp_)
		{
		}
		TF_TreeEdge() : timestamp(INVALID_TIMESTAMP) {}
	};

	// map: [parent] -> { [child] -> relPoseChildWRTParent }
	using pose_tree_t = mrpt::aligned_std_map<
		std::string, mrpt::aligned_std_map<std::string, TF_TreeEdge>>;
	pose_tree_t m_pose_edges_buffer;
};

}  // namespace mrpt::poses
