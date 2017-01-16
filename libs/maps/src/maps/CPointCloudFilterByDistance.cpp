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

using namespace mrpt::poses;
using namespace mrpt::maps;
using namespace mrpt::utils;

void CPointCloudFilterByDistance::filter(
	mrpt::maps::CPointsMap * pc,            //!< [in,out] The input pointcloud, which will be modified upon return after filtering.
	const mrpt::system::TTimeStamp pc_timestamp,          //!< [in] The timestamp of the input pointcloud
	const mrpt::poses::CPose3D * pc_reference_pose        //!< [in] If NULL, the PC is assumed to be given in global coordinates. Otherwise, it will be transformed from local coordinates to global using this transformation.
)
{
	MRPT_START;
	ASSERT_(pc_timestamp!=INVALID_TIMESTAMP);
	ASSERT_(pc != nullptr);

	// 1) Filter:
	// ---------------------
	// get reference, previous PC:
	if (!m_last_frames.empty() && 
		mrpt::system::timeDifference(m_last_frames.rbegin()->first, pc_timestamp) < options.too_old_seconds
		)
	{
		const FrameInfo & prev_pc = m_last_frames.rbegin()->second;

		const size_t N = pc->size();
		std::vector<bool> deletion_mask;
		deletion_mask.assign(N, false);

		// Reference poses of each PC:
		// Previous: prev_pc.pose
		CPose3D cur_pc_pose;
		if (pc_reference_pose) cur_pc_pose = *pc_reference_pose;

		const CPose3D rel_pose = cur_pc_pose - prev_pc.pose;
		// The idea is that we can now find matches between pt{i} in time_{k}, composed with rel_pose
		// with the local points in time_{k-1}.

		for (size_t i = 0; i < N; i++)
		{
			float lx_k, ly_k, lz_k;
			pc->getPointFast(i, lx_k, ly_k, lz_k);

			float lx_km1, ly_km1, lz_km1;
			rel_pose.composePoint(lx_k, ly_k, lz_k, lx_km1, ly_km1, lz_km1);
			
			MRPT_TODO("impl!");
		}

		// Remove points:
		pc->applyDeletionMask(deletion_mask);

	} // we can do filter

	// 2) Add PC to list
	// ---------------------
	{
		FrameInfo fi;
		fi.pc = CSimplePointsMap::Create();
		fi.pc->copyFrom(*pc);
		if (pc_reference_pose) {
			fi.pose = *pc_reference_pose;
		}

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
	min_dist(0.05),
	angle_tolerance( mrpt::utils::DEG2RAD(2)),
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
