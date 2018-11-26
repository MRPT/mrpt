/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <chrono>
#include <string>

namespace mrpt
{
/** Clock that is compatible with MRPT TTimeStamp representation
 * \ingroup mrpt_core_grp
 */
class Clock
{
   public:
	using rep = int64_t;
	// 100-nanoseconds
	using period = std::ratio<1, 10000000>;
	using duration = std::chrono::duration<rep, period>;
	using time_point = std::chrono::time_point<Clock>;

	static constexpr bool is_steady = std::chrono::system_clock::is_steady;

	/** Returns the current time, with the highest resolution available.
	 *  Typically this is better than 1 microsecond. */
	static time_point now() noexcept;
	/** Create a timestamp from its double representation. \sa toDouble */
	static time_point fromDouble(const double t) noexcept;
	/** Converts a timestamp to a UNIX time_t-like number, with fractional part
	 * \sa fromDouble */
	static double toDouble(const time_point t) noexcept;
};
}  // namespace mrpt
