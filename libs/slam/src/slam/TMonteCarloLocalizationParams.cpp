/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "slam-precomp.h"  // Precompiled headerss

#include <mrpt/slam/TMonteCarloLocalizationParams.h>

using namespace mrpt;
using namespace mrpt::slam;
using namespace std;

/*---------------------------------------------------------------
					TMonteCarloLocalizationParams
 ---------------------------------------------------------------*/
TMonteCarloLocalizationParams::TMonteCarloLocalizationParams()
	: metricMaps(), KLD_params()
{
}

TMonteCarloLocalizationParams::TMonteCarloLocalizationParams(
	const TMonteCarloLocalizationParams& o)
{
	*this = o;
}

/** Copy operator: take care of knowing what you do, since this copies pointers.
 */
TMonteCarloLocalizationParams& TMonteCarloLocalizationParams::operator=(
	const TMonteCarloLocalizationParams& o) = default;
