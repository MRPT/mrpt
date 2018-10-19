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
#include <vector>
#include <memory>

namespace mrpt
{
namespace poses
{
class CPose3D;
}
namespace maps
{
class CPointsMap;
}

namespace maps
{
/** Virtual base class for all point-cloud filtering algorithm. See derived
 * classes for implementations.
 * \sa CPointsMap
 * \ingroup mrpt_maps_grp
 */
class CPointCloudFilterBase
{
   public:
	using Ptr = std::shared_ptr<CPointCloudFilterBase>;
	CPointCloudFilterBase();
	virtual ~CPointCloudFilterBase();

	struct TExtraFilterParams
	{
		/** If a pointer is provided to a user-given container, the list of
		 * points to be deleted will be marked here with `true`. */
		std::vector<bool>* out_deletion_mask{nullptr};
		/** (Default:false) If true, only `out_deletion_mask` is filled in, but
		 * the filtered-out points will be not actually removed. */
		bool do_not_delete{false};

		TExtraFilterParams();
	};

	/** Apply the filtering algorithm to the pointcloud. */
	virtual void filter(
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
		TExtraFilterParams* params = nullptr) = 0;
};
}  // namespace maps
}  // namespace mrpt
