/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2025, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/
#pragma once

#include <mrpt/maps/CMetricMap.h>
#include <mrpt/slam/TKLDParams.h>

namespace mrpt::slam
{
/** The struct for passing extra simulation parameters to the prediction stage
 *    when running a particle filter.
 *   \ingroup mrpt_slam_grp
 */
struct TMonteCarloLocalizationParams
{
  TMonteCarloLocalizationParams() = default;

  /** [update stage] Must be set to a metric map used to estimate the
   * likelihood of observations
   */
  mrpt::maps::CMetricMap::ConstPtr metricMap;

  /** [update stage] Alternative way (if metricMap==nullptr): A metric map is
   * supplied for each particle: There must be the same maps here as pose
   * m_particles.
   */
  std::vector<mrpt::maps::CMetricMap::ConstPtr> metricMaps;

  /** Parameters for dynamic sample size, KLD method. */
  TKLDParams KLD_params;
};

}  // namespace mrpt::slam
