/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/maps/CPointCloudFilterBase.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/utils/aligned_containers.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/utils/CLoadableOptions.h>

#include <mrpt/maps/link_pragmas.h>

namespace mrpt
{
	namespace maps
	{
		/** Implementation of pointcloud filtering based on requisities for minimum neigbouring points in both, 
		 *   the current timestamp and a previous one.
		 *
		 * \sa CPointsMap
		  * \ingroup mrpt_maps_grp
		 */
		class MAPS_IMPEXP CPointCloudFilterByDistance : public mrpt::maps::CPointCloudFilterBase
		{
		public:
			// See base docs
			void filter(
				mrpt::maps::CPointsMap * inout_pointcloud,       //!< [in,out] The input pointcloud, which will be modified upon return after filtering.
				const mrpt::system::TTimeStamp pc_timestamp,     //!< [in] The timestamp of the input pointcloud
				const mrpt::poses::CPose3D & pc_reference_pose,  //!< [in] If NULL, the PC is assumed to be given in global coordinates. Otherwise, it will be transformed from local coordinates to global using this transformation.
				TExtraFilterParams * params = nullptr            //!< [in,out] additional in/out parameters
			) MRPT_OVERRIDE;

			struct MAPS_IMPEXP TOptions : public mrpt::utils::CLoadableOptions
			{
				double min_dist;               //!< (Default: 0.05 m)
				double angle_tolerance;        //!< (Default: 2 deg) Stored in rad.
				double too_old_seconds;        //!< (Default: 1 s)
				int    previous_keyframes;     //!< (Default: 1) How many previous keyframes will be compared with the latest pointcloud.
				double max_deletion_ratio;     //!< (Default: 0.4) If the ratio [0,1] of points considered invalid ("deletion") is larger than this ratio, no point will be deleted since it'd be too suspicious and may indicate a failure of this filter.

				TOptions();
				void loadFromConfigFile(const mrpt::utils::CConfigFileBase &source, const std::string &section) MRPT_OVERRIDE; // See base docs
				void saveToConfigFile(mrpt::utils::CConfigFileBase &c, const std::string &section) const MRPT_OVERRIDE;
			};

			TOptions options;

		private:
			struct MAPS_IMPEXP FrameInfo
			{
				mrpt::poses::CPose3D  pose;
				mrpt::maps::CSimplePointsMapPtr pc;

				MRPT_MAKE_ALIGNED_OPERATOR_NEW
			};

			mrpt::aligned_containers<mrpt::system::TTimeStamp, FrameInfo>::map_t   m_last_frames;
		};

	}
} // End of namespace
