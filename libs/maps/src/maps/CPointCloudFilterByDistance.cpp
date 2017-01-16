/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "maps-precomp.h" // Precomp header

#include <mrpt/maps/CPointCloudFilterByDistance.h>
#include <mrpt/utils/CConfigFileBase.h>

using namespace mrpt::maps;
using namespace mrpt::utils;

void CPointCloudFilterByDistance::filter(
	mrpt::maps::CPointsMap * pc,       //!< [in,out] The input pointcloud, which will be modified upon return after filtering.
	const mrpt::system::TTimeStamp pc_timestamp,     //!< [in] The timestamp of the input pointcloud
	const mrpt::poses::CPose3D & cur_pc_pose,  //!< [in] If NULL, the PC is assumed to be given in global coordinates. Otherwise, it will be transformed from local coordinates to global using this transformation.
	TExtraFilterParams * params                      //!< [in,out] additional in/out parameters
)
{
	using namespace mrpt::poses;
	using namespace mrpt::math;
	using mrpt::utils::square;

	MRPT_START;
	ASSERT_(pc_timestamp!=INVALID_TIMESTAMP);
	ASSERT_(pc != nullptr);

	CSimplePointsMapPtr original_pc = CSimplePointsMap::Create();
	original_pc->copyFrom(*pc);

	// 1) Filter:
	// ---------------------
	const size_t N = pc->size();
	std::vector<bool> deletion_mask;
	deletion_mask.assign(N, false);

	// get reference, previous PC:
	if (!m_last_frames.empty() && 
		mrpt::system::timeDifference(m_last_frames.rbegin()->first, pc_timestamp) < options.too_old_seconds
		)
	{
		const FrameInfo & prev_pc = m_last_frames.rbegin()->second;

		// Reference poses of each PC:
		// Previous: prev_pc.pose
		const CPose3D rel_pose = cur_pc_pose - prev_pc.pose;
		// The idea is that we can now find matches between pt{i} in time_{k}, composed with rel_pose
		// with the local points in time_{k-1}.

		for (size_t i = 0; i < N; i++)
		{
			// get i-th point in time=k:
			TPoint3Df ptf_k;
			pc->getPointFast(i, ptf_k.x, ptf_k.y, ptf_k.z);
			const TPoint3D pt_k = TPoint3D(ptf_k);

			// Point, referred to time=k-1 frame of reference
			TPoint3D pt_km1;
			rel_pose.composePoint(pt_k, pt_km1);

			// Look for neighbors in "time=k"
			std::vector<TPoint3D> neig_k;
			std::vector<float>    neig_sq_dist_k;
			pc->kdTreeNClosestPoint3D(pt_k, 2 /*num queries*/, neig_k, neig_sq_dist_k);

			// Look for neighbors in "time=k-1"
			std::vector<TPoint3D> neig_km1;
			std::vector<float>    neig_sq_dist_km1;
			prev_pc.pc->kdTreeNClosestPoint3D(pt_km1, 1 /*num queries*/, neig_km1, neig_sq_dist_km1);

			// Rule:
			// we must have at least 1 neighbor in t=k, and 1 neighbor in t=k-1
			const double max_allowed_dist_sq = square(options.min_dist + options.angle_tolerance * pt_k.norm());

			const bool ok_t   = neig_k.size() > 1 && neig_sq_dist_k[1] < max_allowed_dist_sq;
			const bool ok_tm1 = neig_sq_dist_km1.size() >= 1 && neig_sq_dist_km1[0] < max_allowed_dist_sq;

			// Delete?
			deletion_mask[i] = !(ok_t && ok_tm1);
		}

		// Remove points:
		if (params == nullptr || params->do_not_delete == false) {
			pc->applyDeletionMask(deletion_mask);
		}
	} // we can do filter

	if (params != nullptr && params->out_deletion_mask != nullptr) {
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
	for (auto it = m_last_frames.begin(); it != m_last_frames.end(); )
	{
		if (mrpt::system::timeDifference(it->first, pc_timestamp) > options.too_old_seconds) {
			it = m_last_frames.erase(it);
		}
		else {
			++it;
		}
	}

	MRPT_END;
}

CPointCloudFilterByDistance::TOptions::TOptions() :
	min_dist(0.10),
	angle_tolerance( mrpt::utils::DEG2RAD(5)),
	too_old_seconds(1.0)
{
}

void CPointCloudFilterByDistance::TOptions::loadFromConfigFile(const mrpt::utils::CConfigFileBase &c, const std::string &s)
{
	MRPT_LOAD_CONFIG_VAR(min_dist, double, c, s);
	MRPT_LOAD_CONFIG_VAR_DEGREES(angle_tolerance, c, s);
	MRPT_LOAD_CONFIG_VAR(too_old_seconds, double, c, s);
}

void CPointCloudFilterByDistance::TOptions::dumpToTextStream(mrpt::utils::CStream &out) const
{
	LOADABLEOPTS_DUMP_VAR(min_dist, double);
	LOADABLEOPTS_DUMP_VAR_DEG(angle_tolerance);
	LOADABLEOPTS_DUMP_VAR(too_old_seconds, double);
}
