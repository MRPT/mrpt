/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <chrono>

namespace mrpt
{
/** C++11-clock that is compatible with MRPT TTimeStamp representation
 *
 * State-aware methods in this class are thread-safe.
 *
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

	/** Returns the current time using the currently selected Clock source.
	 * \sa setActiveClock(), mrpt::Clock::Source  */
	static time_point now() noexcept;
	/** Create a timestamp from its double representation. \sa toDouble */
	static time_point fromDouble(const double t) noexcept;
	/** Converts a timestamp to a UNIX time_t-like number, with fractional part
	 * \sa fromDouble */
	static double toDouble(const time_point t) noexcept;

	/** Options for setting the source of all timestamps across MRPT:
	 * setActiveClock(), now()
	 */
	enum Source
	{
		/** Realtime: POSIX `CLOCK_REALTIME`. The highest resolution available
		 *  Typically this is better than 1 microsecond. Selected by default.
		 */
		Realtime = 0,

		/** Monotonic: POSIX `CLOCK_MONOTONIC`. Only available in Linux systems.
		 */
		Monotonic,

		/** Simulated time: Manual control of time via setSimulatedTime()
		 * [New in MRPT 2.1.1]
		 */
		Simulated
	};

	/** Changes the selected clock to get time from when calling now().
	 * Default: Realtime.
	 *
	 * Monotonic is only available on Linux systems.
	 * RealTime and Simulated are available on any platform.
	 *
	 * It is strongly recommended to call setSimulatedTime()
	 * before setting the clock source to Simulated to ensure that any
	 * subsequent call to now(), perhaps in a parallel thread, does not
	 * return an undefined time_point value.
	 */
	static void setActiveClock(const Source s);

	/** Returns the currently selected clock. */
	static Source getActiveClock();

	/** Monotonic clock *might* drift over time with respect to Realtime.
	 * The first time a time is requested to now() with Monotonic clock enabled,
	 * MRPT internally saves the current values of both, Realtime and Monotonic
	 * clocks, then use the difference in all subsequent calls to set Monotonic
	 * timepoints in the same epoch than Realtime.
	 *
	 * By explicitly calling this static method, a user is able to force a
	 * resynchrnization between the two clocks. Potentially useful for systems
	 * that run for very long periods of time without interruption.
	 *
	 * \return The mismatch between the former and the new estimations of the
	 * epochs differences between the two clocks, in units of nanoseconds.
	 *
	 * \note This method is ignored in non-Linux systems.
	 */
	static int64_t resetMonotonicToRealTimeEpoch() noexcept;

	/** Returns the number of nanoseconds that are added to the output of the
	 * POSIX `CLOCK_MONOTONIC` to make timestamps match the epoch of POSIX
	 * `CLOCK_REALTIME`. */
	static uint64_t getMonotonicToRealtimeOffset();

	/** When setActiveClock() is set to `Simulated`, sets the simulated time
	 * that will be returned in subsequent calls to now().
	 * [New in MRPT 2.1.1]
	 */
	static void setSimulatedTime(const time_point& t);
};
}  // namespace mrpt
