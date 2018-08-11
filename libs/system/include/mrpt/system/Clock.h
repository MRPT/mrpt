/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/core/Clock.h>
#include <chrono>
#include <string>

namespace mrpt::system
{

// Clock that is compatible with TTimeStamp representation
using Clock = mrpt::core::Clock;

/** Returns the time difference from t1 to t2 (positive if t2 is posterior to
 * t1), in seconds \sa secondsToTimestamp */
double timeDifference(
	const mrpt::system::Clock::time_point &t_first,
	const mrpt::system::Clock::time_point &t_later);

/** Convert a timestamp into this textual form (in local time): HH:MM:SS.MMMMMM
 */
inline Clock::time_point toTimePoint(int64_t time)
{
	return Clock::time_point(Clock::duration(time));
}

std::string timeLocalToString(
	const mrpt::system::Clock::time_point &t, unsigned int secondFractionDigits = 6);

/** Transform from TTimeStamp to standard "time_t" (actually a double number, it
  * can contain fractions of seconds).
  * This function is just an (inline) alias of timestampTotime_t(), with a more
  * significant name.
  * \sa time_tToTimestamp, secondsToTimestamp
  */
double timestampTotime_t(const mrpt::system::Clock::time_point t);


}
