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
#include <mrpt/system/datetime.h>

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

std::string timeLocalToString(
	const mrpt::system::Clock::time_point &t, unsigned int secondFractionDigits)
{
	return timeLocalToString(t.time_since_epoch().count(), secondFractionDigits);
}

double timestampTotime_t(const mrpt::system::Clock::time_point t)
{
        return double(t.time_since_epoch().count() - UINT64_C(116444736) * UINT64_C(1000000000)) / 10000000.0;
}

}
