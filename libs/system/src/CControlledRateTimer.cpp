/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "system-precomp.h"  // Precompiled headers
//
#include <mrpt/core/bits_math.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/system/CControlledRateTimer.h>

#include <cmath>  // std::abs(double)

using namespace mrpt::system;

CControlledRateTimer::CControlledRateTimer(const double rate_hz)
	: mrpt::system::COutputLogger("CControlledRateTimer")
{
	m_tic.Tic();
	setRate(rate_hz);
}
void CControlledRateTimer::setRate(const double rate_hz)
{
	if (rate_hz == m_rate_hz) return;

	ASSERT_GT_(rate_hz, 0.0);
	m_rate_hz = rate_hz;
	m_lastControlError = 0;
	m_currentEstimatedRate = m_rate_hz;
	m_lastTic = 0;
}

bool CControlledRateTimer::sleep()
{
	const bool validRateEstimate = internalUpdateRateEstimate();

	const double rawError = m_rate_hz - m_currentEstimatedRate;

	const double controlError =
		mrpt::saturate_val(rawError, -0.2 * m_rate_hz, 0.2 * m_rate_hz);

	if (std::abs(rawError) / m_rate_hz > m_followErrorRatioForWarning)
	{
		MRPT_LOG_THROTTLE_WARN_FMT(
			2.0,
			"Cannot run at the expected rate: actual_rate=%.03f Hz "
			"desired_rate=%.03f Hz",
			m_currentEstimatedRate, m_rate_hz);
	}

	// Trapezoidal approx. of PI(s) controller equation
	// integral e(t)->s=(2/T)*(1-z^-1)/(1+z^-1)
	// derivative e(t)->s=(1/T)*(1-z^-1)
	const double q0 = m_Kp * (1 + 1.0 / (m_rate_hz * 2 * m_Ti));
	const double q1 = m_Kp * (-1 + 1.0 / (m_rate_hz * 2 * m_Ti));

	double newRate;
	if (validRateEstimate)
	{
		newRate =
			m_ratetimer.rate() + q0 * controlError + q1 * m_lastControlError;
	}
	else
	{
		newRate = m_rate_hz;
	}

	// Set control output:
	m_ratetimer.setRate(newRate);

	m_lastControlError = controlError;

	return m_ratetimer.sleep();
}

bool CControlledRateTimer::internalUpdateRateEstimate()
{
	bool valid = false;
	const double t_now = m_tic.Tac();

	if (m_lastTic > 0 && t_now > m_lastTic)
	{
		const double measuredPeriod = t_now - m_lastTic;
		m_lastRawRate = (1.0 / measuredPeriod);

		// Filter:
		m_currentEstimatedRate = m_lowPass_a0 * m_currentEstimatedRate +
								 (1.0 - m_lowPass_a0) * m_lastRawRate;
		valid = true;
	}

	m_lastTic = t_now;
	return valid;
}
