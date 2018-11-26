/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "core-precomp.h"  // Precompiled headers

#include <mrpt/core/Clock.h>

#ifdef _WIN32
#include <windows.h>
#else
#include <sys/time.h>  // timeval
#endif

#include <ctime>  // clock_gettime

static uint64_t getCurrentTime()
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
	clock_gettime(CLOCK_REALTIME, &tim);
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
