/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | Copyright (c) 2012, University of Almeria                                 |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
   +---------------------------------------------------------------------------+ */
#ifndef TMonteCarloLocalizationParams_H
#define TMonteCarloLocalizationParams_H

#include <mrpt/slam/CMetricMap.h>
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
			CMetricMap			*metricMap;

			/** [update stage] Alternative way (if metricMap==NULL): A metric map is supplied for each particle: There must be the same maps here as pose m_particles.
				*/
			TMetricMapList		metricMaps;

			TKLDParams			KLD_params; //!< Parameters for dynamic sample size, KLD method.
		};

	} // End of namespace
} // End of namespace

#endif
