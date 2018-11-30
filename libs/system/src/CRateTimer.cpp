/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "system-precomp.h"  // Precompiled headers

#include <mrpt/system/CRateTimer.h>
#include <mrpt/core/exceptions.h>
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
	const int64_t wait_tim_us = 1000000L * (period - elapsed_tim);
	if (elapsed_tim > period)
	{
		m_tictac.Tic();
		return false;
	}
	std::this_thread::sleep_for(std::chrono::microseconds(wait_tim_us));
	m_tictac.Tic();
	return true;
}
