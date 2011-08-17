/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
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
