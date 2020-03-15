/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "core-precomp.h"  // Precompiled headers

#include <mrpt/core/Clock.h>
#include <mrpt/core/exceptions.h>

#ifdef _WIN32
#include <windows.h>
#else
#include <sys/time.h>  // timeval
#endif

#include <ctime>  // clock_gettime
#include <iostream>

static mrpt::Clock::Source selectedClock = mrpt::Clock::Source::Realtime;

#if !defined(_WIN32)
inline uint64_t as_nanoseconds(const struct timespec& ts)
{
	return ts.tv_sec * static_cast<uint64_t>(1000000000L) + ts.tv_nsec;
}
inline void from_nanoseconds(const uint64_t ns, struct timespec& ts)
{
	ts.tv_sec = (ns / static_cast<uint64_t>(1000000000L));
	ts.tv_nsec = (ns % static_cast<uint64_t>(1000000000L));
}
#endif
struct MonotonicToRealtimeEpoch
{
	uint64_t monotonic_ns = 0;
	uint64_t realtime_ns = 0;
	uint64_t rt2mono_diff = 0;
};
static MonotonicToRealtimeEpoch m2r_epoch;
static bool monotonic_epoch_init = false;

static uint64_t getCurrentTime() noexcept
{
#ifdef _WIN32
	FILETIME t;
	GetSystemTimeAsFileTime(&t);
	return (((uint64_t)t.dwHighDateTime) << 32) | ((uint64_t)t.dwLowDateTime);
#else
#if defined(__APPLE__)
	struct timeval tv;
	timespec tim{0, 0};
	gettimeofday(&tv, nullptr);
	tim.tv_sec = tv.tv_sec;
	tim.tv_nsec = tv.tv_usec * 1000;
#else
	timespec tim{0, 0};

	switch (selectedClock)
	{
		case mrpt::Clock::Source::Realtime:
		{
			// Just get the time and that is it:
			clock_gettime(CLOCK_REALTIME, &tim);
		}
		break;
		case mrpt::Clock::Source::Monotonic:
		{
			// Get the realtime clock reference timepoint, the first time we run
			// this:
			if (!monotonic_epoch_init)
				mrpt::Clock::resetMonotonicToRealTimeEpoch();

			// get the monotonic clock:
			clock_gettime(CLOCK_MONOTONIC, &tim);

			// correct the clock epoch:
			const uint64_t sum = as_nanoseconds(tim) + m2r_epoch.rt2mono_diff;
			from_nanoseconds(sum, tim);
		}
		break;
	};
#endif

	// Convert to TTimeStamp 100-nanoseconds representation:
	return uint64_t(tim.tv_sec) * UINT64_C(10000000) +
		   UINT64_C(116444736) * UINT64_C(1000000000) + tim.tv_nsec / 100;
#endif
}

mrpt::Clock::time_point mrpt::Clock::now() noexcept
{
	return time_point(duration(getCurrentTime()));
}

mrpt::Clock::time_point mrpt::Clock::fromDouble(const double t) noexcept
{
	return mrpt::Clock::time_point(mrpt::Clock::duration(
		uint64_t(t * 10000000.0) + UINT64_C(116444736) * UINT64_C(1000000000)));
}

// Convert to time_t UNIX timestamp, with fractional part.
double mrpt::Clock::toDouble(const mrpt::Clock::time_point t) noexcept
{
	return double(
			   t.time_since_epoch().count() -
			   UINT64_C(116444736) * UINT64_C(1000000000)) /
		   10000000.0;
}

void mrpt::Clock::setActiveClock(const Source s)
{
	ASSERT_(
		s == mrpt::Clock::Source::Realtime ||
		s == mrpt::Clock::Source::Monotonic);
	selectedClock = s;
}

mrpt::Clock::Source mrpt::Clock::getActiveClock() { return selectedClock; }

int64_t mrpt::Clock::resetMonotonicToRealTimeEpoch() noexcept
{
#if !defined(_WIN32)
	timespec tim_rt{0, 0}, tim_mono{0, 0};
	clock_gettime(CLOCK_REALTIME, &tim_rt);
	clock_gettime(CLOCK_MONOTONIC, &tim_mono);
	m2r_epoch.monotonic_ns = as_nanoseconds(tim_mono);
	m2r_epoch.realtime_ns = as_nanoseconds(tim_rt);

	// Save the difference, which is what matters:
	const auto old_diff = m2r_epoch.rt2mono_diff;
	if (m2r_epoch.realtime_ns < m2r_epoch.monotonic_ns)
	{
		std::cerr << "[mrpt::Clock::resetMonotonicToRealTimeEpoch] CRITICAL: "
					 "Unreliable values: realtime_ns="
				  << m2r_epoch.realtime_ns
				  << " should be larger than monotonic_ns="
				  << m2r_epoch.monotonic_ns << "\n";
	}
	m2r_epoch.rt2mono_diff = m2r_epoch.realtime_ns - m2r_epoch.monotonic_ns;

	const int64_t err = monotonic_epoch_init
							? (static_cast<int64_t>(m2r_epoch.rt2mono_diff) -
							   static_cast<int64_t>(old_diff))
							: 0;

	monotonic_epoch_init = true;
	return err;
#else
	monotonic_epoch_init = true;
	return 0;
#endif
}

uint64_t mrpt::Clock::getMonotonicToRealtimeOffset()
{
	if (!monotonic_epoch_init) mrpt::Clock::resetMonotonicToRealTimeEpoch();
	return m2r_epoch.rt2mono_diff;
}
