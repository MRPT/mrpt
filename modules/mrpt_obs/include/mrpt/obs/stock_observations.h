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

#include <mrpt/img/CImage.h>
#include <mrpt/obs/obs_frwds.h>

/** A few stock observations for easy reuse in unit tests, examples, etc.
 * \ingroup mrpt_obs_grp
 */
namespace mrpt::obs::stock_observations
{
/** Example 2D lidar scans (form a venerable SICK LMS200).
 * Implemented indices: 0,1.
 * \ingroup mrpt_obs_grp
 */
void example2DRangeScan(mrpt::obs::CObservation2DRangeScan& s, int i = 0);

/** Example images (an 800x640 image pair from a Bumblebee 1)
 * Implemented indices: 0,1.
 * \ingroup mrpt_obs_grp
 */
void exampleImage(mrpt::img::CImage& im, int i = 0);

}  // namespace mrpt::obs::stock_observations
