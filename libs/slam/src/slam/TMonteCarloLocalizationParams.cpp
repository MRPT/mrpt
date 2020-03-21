/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
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
