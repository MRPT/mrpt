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

using namespace mrpt::core;

static uint64_t getCurrentTime()
{
#ifdef _WIN32
	FILETIME t;
	GetSystemTimeAsFileTime(&t);
	return (((uint64_t)t.dwHighDateTime) << 32) | ((uint64_t)t.dwLowDateTime);
#elif defined(__APPLE__)
	struct timeval tv;
	timespec tim;
	gettimeofday(&tv, nullptr);
	tim.tv_sec = tv.tv_sec;
	tim.tv_nsec = tv.tv_usec * 1000;
#else
	timespec tim;
	clock_gettime(CLOCK_REALTIME, &tim);
#endif

	// Convert to TTimeStamp 100-nanoseconds representation:
	return uint64_t(tim.tv_sec) * UINT64_C(10000000) +
		   UINT64_C(116444736) * UINT64_C(1000000000) + tim.tv_nsec / 100;
}

Clock::time_point Clock::now() noexcept
{
	return time_point(duration(getCurrentTime()));
}
