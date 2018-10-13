/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/maps/CPointCloudFilterBase.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/core/aligned_std_map.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/config/CLoadableOptions.h>

namespace mrpt::maps
{
/** Implementation of pointcloud filtering based on requisities for minimum
 * neigbouring points in both,
 *   the current timestamp and a previous one.
 *
 * \sa CPointsMap
 * \ingroup mrpt_maps_grp
 */
class CPointCloudFilterByDistance : public mrpt::maps::CPointCloudFilterBase
{
   public:
	// See base docs
	void filter(
		/** [in,out] The input pointcloud, which will be modified upon
		   return after filtering. */
		mrpt::maps::CPointsMap* inout_pointcloud,
		/** [in] The timestamp of the input pointcloud */
		const mrpt::system::TTimeStamp pc_timestamp,
		/** [in] If nullptr, the PC is assumed to be given in global
		   coordinates. Otherwise, it will be transformed from local
		   coordinates to global using this transformation. */
		const mrpt::poses::CPose3D& pc_reference_pose,
		/** [in,out] additional in/out parameters */
		TExtraFilterParams* params = nullptr) override;

	struct TOptions : public mrpt::config::CLoadableOptions
	{
		/** (Default: 0.05 m) */
		double min_dist{0.10};
		/** (Default: 2 deg) Stored in rad. */
		double angle_tolerance;
		/** (Default: 1 s) */
		double too_old_seconds{1.0};
		/** (Default: 1) How many previous keyframes will be compared with the
		 * latest pointcloud. */
		int previous_keyframes{1};
		/** (Default: 0.4) If the ratio [0,1] of points considered invalid
		 * ("deletion") is larger than this ratio, no point will be deleted
		 * since it'd be too suspicious and may indicate a failure of this
		 * filter. */
		double max_deletion_ratio{.4};

		TOptions();
		void loadFromConfigFile(
			const mrpt::config::CConfigFileBase& source,
			const std::string& section) override;  // See base docs
		void saveToConfigFile(
			mrpt::config::CConfigFileBase& c,
			const std::string& section) const override;
	};

	TOptions options;

   private:
	struct FrameInfo
	{
		mrpt::poses::CPose3D pose;
		mrpt::maps::CSimplePointsMap::Ptr pc;

		MRPT_MAKE_ALIGNED_OPERATOR_NEW
	};

	mrpt::aligned_std_map<mrpt::system::TTimeStamp, FrameInfo> m_last_frames;
};
}  // namespace mrpt::maps
