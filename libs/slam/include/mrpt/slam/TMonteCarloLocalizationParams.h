/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef TMonteCarloLocalizationParams_H
#define TMonteCarloLocalizationParams_H

#include <mrpt/maps/CMetricMap.h>
#include <mrpt/slam/TKLDParams.h>

#include <mrpt/slam/link_pragmas.h>

namespace mrpt
{
	namespace slam
	{
		/** The struct for passing extra simulation parameters to the prediction stage
			*    when running a particle filter.
			*   \ingroup mrpt_slam_grp
			*/
		struct SLAM_IMPEXP TMonteCarloLocalizationParams
		{
			/** Default settings method.
				*/
			TMonteCarloLocalizationParams();

			/** Copy constructor: take care of knowing what you do, since this copies pointers. */
			TMonteCarloLocalizationParams( const TMonteCarloLocalizationParams &o );

			/** Copy operator: take care of knowing what you do, since this copies pointers. */
			TMonteCarloLocalizationParams & operator =(const TMonteCarloLocalizationParams &o);

			/** [update stage] Must be set to a metric map used to estimate the likelihood of observations
				*/
			mrpt::maps::CMetricMap *metricMap;

			/** [update stage] Alternative way (if metricMap==NULL): A metric map is supplied for each particle: There must be the same maps here as pose m_particles.
				*/
			mrpt::maps::TMetricMapList		metricMaps;

			TKLDParams			KLD_params; //!< Parameters for dynamic sample size, KLD method.
		};

	} // End of namespace
} // End of namespace

#endif
