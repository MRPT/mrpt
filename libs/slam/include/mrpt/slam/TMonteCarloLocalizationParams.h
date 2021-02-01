/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
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
