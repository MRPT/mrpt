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
#pragma once

#include <mrpt/math/TPoint3D.h>
#include <mrpt/vision/CFeatureExtraction.h>

#include <cstdint>

namespace mrpt::maps
{
/** A minimal representation of a visual landmark.
 * \ingroup mrpt_slam_grp
 */
struct CLandmark
{
  using TLandmarkID = int64_t;

  TLandmarkID ID{0};

  // Removed in mrpt v3: std::vector<mrpt::vision::CFeature> features;

  mrpt::math::TPoint3D pose_mean{0, 0, 0};

  double pose_cov_11{0}, pose_cov_22{0}, pose_cov_33{0};
  double pose_cov_12{0}, pose_cov_13{0}, pose_cov_23{0};

  size_t seenTimesCount{0};
};

}  // namespace mrpt::maps
