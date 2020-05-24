/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "system-precomp.h"  // Precompiled headers

#include <mrpt/config.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/system/CRateTimer.h>
#include <chrono>
#include <thread>

#if defined(MRPT_OS_LINUX)
#include <time.h>  // clock_nanosleep()
#endif

using namespace mrpt::system;

CRateTimer::CRateTimer(const double rate_hz) { setRate(rate_hz); }
void CRateTimer::setRate(const double rate_hz)
{
	ASSERT_ABOVE_(rate_hz, 0.0);
	m_rate_hz = rate_hz;
}
bool CRateTimer::sleep()
{
	const double elapsed_tim = m_tictac.Tac();
	const double period = 1.0 / m_rate_hz;
	const int64_t wait_tim_ns =
		static_cast<int64_t>(1000000000L * (period - elapsed_tim));
	if (elapsed_tim > period)
	{
		m_tictac.Tic();
		return false;
	}

#if !defined(MRPT_OS_LINUX)
	std::this_thread::sleep_for(std::chrono::nanoseconds(wait_tim_ns));
#else
	timespec ts{0, 0}, ts_remainer{0, 0};
	ts.tv_sec = wait_tim_ns / 1000000000;
	ts.tv_nsec = wait_tim_ns % 1000000000;

	for (;;)
	{
		int err = clock_nanosleep(CLOCK_MONOTONIC, 0, &ts, &ts_remainer);
		if (err == 0) break;  // all good
		if (err == EINTR)
		{
			ts = ts_remainer;
			continue;
		}
		fprintf(
			stderr, "[CRateTimer::sleep] Error %i in clock_nanosleep()\n", err);
		break;
	}

#endif
	m_tictac.Tic();
	return true;
}
