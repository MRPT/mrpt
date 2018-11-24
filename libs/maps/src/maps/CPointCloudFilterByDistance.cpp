/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "maps-precomp.h"  // Precomp header

#include <mrpt/maps/CPointCloudFilterByDistance.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/core/aligned_std_vector.h>

using namespace mrpt::maps;

void CPointCloudFilterByDistance::filter(
	/** [in,out] The input pointcloud, which will be modified upon return after
	   filtering. */
	mrpt::maps::CPointsMap* pc,
	/** [in] The timestamp of the input pointcloud */
	const mrpt::system::TTimeStamp pc_timestamp,
	/** [in] If nullptr, the PC is assumed to be given in global coordinates.
	   Otherwise, it will be transformed from local coordinates to global using
	   this transformation. */
	const mrpt::poses::CPose3D& cur_pc_pose,
	/** [in,out] additional in/out parameters */
	TExtraFilterParams* params)
{
	using namespace mrpt::poses;
	using namespace mrpt::math;
	using mrpt::square;

	MRPT_START;
	ASSERT_(pc_timestamp != INVALID_TIMESTAMP);
	ASSERT_(pc != nullptr);

	auto original_pc = CSimplePointsMap::Create();
	(*original_pc) = (*pc);  // make deep copy

	// 1) Filter:
	// ---------------------
	const size_t N = pc->size();
	std::vector<bool> deletion_mask;
	deletion_mask.assign(N, false);
	size_t del_count = 0;

	// get reference, previous PC:
	ASSERT_(options.previous_keyframes >= 1);
	bool can_do_filter = true;

	std::vector<FrameInfo*> prev_pc;  // (options.previous_keyframes, nullptr);
	{
		auto it = m_last_frames.rbegin();
		for (int i = 0;
			 i < options.previous_keyframes && it != m_last_frames.rend();
			 ++i, ++it)
		{
			prev_pc.push_back(&it->second);
		}
	}

	if (prev_pc.size() < static_cast<size_t>(options.previous_keyframes))
	{
		can_do_filter = false;
	}
	else
	{
		for (int i = 0; can_do_filter && i < options.previous_keyframes; ++i)
		{
			if (mrpt::system::timeDifference(
					m_last_frames.rbegin()->first, pc_timestamp) >
				options.too_old_seconds)
			{
				can_do_filter = false;  // A required keyframe is too old
				break;
			}
		}
	}

	if (can_do_filter)
	{
		// Reference poses of each PC:
		// Previous: prev_pc.pose
		mrpt::aligned_std_vector<CPose3D> rel_poses;
		for (int k = 0; k < options.previous_keyframes; ++k)
		{
			const CPose3D rel_pose = cur_pc_pose - prev_pc[k]->pose;
			rel_poses.push_back(rel_pose);
		}

		// The idea is that we can now find matches between pt{i} in time_{k},
		// composed with rel_pose
		// with the local points in time_{k-1}.

		std::vector<TPoint3D> pt_km1(options.previous_keyframes);

		for (size_t i = 0; i < N; i++)
		{
			// get i-th point in time=k:
			TPoint3Df ptf_k;
			pc->getPointFast(i, ptf_k.x, ptf_k.y, ptf_k.z);
			const TPoint3D pt_k = TPoint3D(ptf_k);

			// Point, referred to time=k-1 frame of reference
			for (int k = 0; k < options.previous_keyframes; ++k)
				rel_poses[k].composePoint(pt_k, pt_km1[k]);

			// Look for neighbors in "time=k"
			std::vector<TPoint3D> neig_k;
			std::vector<float> neig_sq_dist_k;
			pc->kdTreeNClosestPoint3D(
				pt_k, 2 /*num queries*/, neig_k, neig_sq_dist_k);

			// Look for neighbors in "time=k-i"
			std::vector<TPoint3D> neig_kmi(options.previous_keyframes);
			std::vector<float> neig_sq_dist_kmi(
				options.previous_keyframes, std::numeric_limits<float>::max());

			for (int k = 0; k < options.previous_keyframes; ++k)
			{
				for (int prev_tim_idx = 0;
					 prev_tim_idx < options.previous_keyframes; prev_tim_idx++)
				{
					if (prev_pc[prev_tim_idx]->pc->size() > 0)
					{
						prev_pc[prev_tim_idx]->pc->kdTreeClosestPoint3D(
							pt_km1[prev_tim_idx], neig_kmi[prev_tim_idx],
							neig_sq_dist_kmi[prev_tim_idx]);
					}
				}
			}

			// Rule:
			// we must have at least 1 neighbor in t=k, and 1 neighbor in t=k-i
			const double max_allowed_dist_sq = square(
				options.min_dist + options.angle_tolerance * pt_k.norm());

			bool ok_total = true;
			const bool ok_t =
				neig_k.size() > 1 && neig_sq_dist_k[1] < max_allowed_dist_sq;
			ok_total = ok_total && ok_t;

			for (int k = 0; k < options.previous_keyframes; ++k)
			{
				const bool ok_tm1 = neig_sq_dist_kmi[k] < max_allowed_dist_sq;
				ok_total = ok_total && ok_tm1;
			}

			// Delete?
			const bool do_delete = !(ok_total);
			deletion_mask[i] = do_delete;

			if (do_delete) del_count++;
		}

		// Remove points:
		if ((params == nullptr || params->do_not_delete == false) && N > 0 &&
			del_count / double(N) <
				options.max_deletion_ratio  // If we are deleting too many
			// points, it may be that the filter
			// is plainly wrong
		)
		{
			pc->applyDeletionMask(deletion_mask);
		}
	}  // we can do filter

	if (params != nullptr && params->out_deletion_mask != nullptr)
	{
		*params->out_deletion_mask = deletion_mask;
	}

	// 2) Add PC to list
	// ---------------------
	{
		FrameInfo fi;
		fi.pc = original_pc;
		fi.pose = cur_pc_pose;

		m_last_frames[pc_timestamp] = fi;
	}

	// 3) Remove too-old PCs.
	// ---------------------
	for (auto it = m_last_frames.begin(); it != m_last_frames.end();)
	{
		if (mrpt::system::timeDifference(it->first, pc_timestamp) >
			options.too_old_seconds)
		{
			it = m_last_frames.erase(it);
		}
		else
		{
			++it;
		}
	}

	MRPT_END;
}

CPointCloudFilterByDistance::TOptions::TOptions()
	: angle_tolerance(mrpt::DEG2RAD(5))

{
}

void CPointCloudFilterByDistance::TOptions::loadFromConfigFile(
	const mrpt::config::CConfigFileBase& c, const std::string& s)
{
	MRPT_LOAD_CONFIG_VAR(min_dist, double, c, s);
	MRPT_LOAD_CONFIG_VAR_DEGREES(angle_tolerance, c, s);
	MRPT_LOAD_CONFIG_VAR(too_old_seconds, double, c, s);
	MRPT_LOAD_CONFIG_VAR(previous_keyframes, int, c, s);
	MRPT_LOAD_CONFIG_VAR(max_deletion_ratio, double, c, s);
}

void CPointCloudFilterByDistance::TOptions::saveToConfigFile(
	mrpt::config::CConfigFileBase& c, const std::string& s) const
{
	MRPT_SAVE_CONFIG_VAR_COMMENT(min_dist, "");
	MRPT_SAVE_CONFIG_VAR_DEGREES_COMMENT(
		"angle_tolerance", angle_tolerance, "");
	MRPT_SAVE_CONFIG_VAR_COMMENT(too_old_seconds, "");
	MRPT_SAVE_CONFIG_VAR_COMMENT(
		previous_keyframes,
		"(Default: 1) How many previous keyframes will be compared with the "
		"latest pointcloud.");
	MRPT_SAVE_CONFIG_VAR_COMMENT(
		max_deletion_ratio,
		"(Default: 0.4) If the ratio [0,1] of points considered invalid "
		"(`deletion` ) is larger than this ratio, no point will be deleted "
		"since it'd be too suspicious and may indicate a failure of this "
		"filter.");
}
