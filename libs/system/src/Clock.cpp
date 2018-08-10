/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "system-precomp.h"  // Precompiled headers

#include <mrpt/system/Clock.h>
#include <mrpt/core/exceptions.h>

namespace mrpt::system
{
/*---------------------------------------------------------------
					timeDifference
  ---------------------------------------------------------------*/
double timeDifference(
	const mrpt::system::Clock::time_point &t1, const mrpt::system::Clock::time_point &t2)
{
	MRPT_START

	return (t1 - t2).count() / 10000000.0;

	MRPT_END
}

}
