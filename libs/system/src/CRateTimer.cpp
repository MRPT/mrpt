/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "system-precomp.h"  // Precompiled headers

#include <mrpt/core/exceptions.h>
#include <mrpt/system/CRateTimer.h>
#include <chrono>
#include <thread>

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
	const int64_t wait_tim_us =
		static_cast<int64_t>(1000000L * (period - elapsed_tim));
	if (elapsed_tim > period)
	{
		m_tictac.Tic();
		return false;
	}
	std::this_thread::sleep_for(std::chrono::microseconds(wait_tim_us));
	m_tictac.Tic();
	return true;
}
