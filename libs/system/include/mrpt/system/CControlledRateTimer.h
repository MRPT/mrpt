/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/core/exceptions.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/system/CRateTimer.h>
#include <mrpt/system/CTicTac.h>

namespace mrpt::system
{
/** A class for calling sleep() in a loop, such that the amount of sleep time
 * will be computed to make the loop run at the desired rate (in Hz).
 * This class implements a PI controller on top of a vanilla CRateTimer object,
 * ensuring a high accuracy in achieved execution rates. Note that this is done
 * by setting a slightly-higher rate ("control action") to the internal
 * CRateTimer, such that the error between the user-provided expected rate and
 * the actual measured rate (low-pass filtered) is decreased by means of a PI
 * controller.
 *
 * Note that rates higher than a few kHz are not attainable in all CPUs and/or
 * kernel versions. Find below some graphs illustrating how this class tries to
 * achieve a constant setpoint rate (given by the user), reacting to changes in
 * the setpoint values:
 *
 *  ![controlled rate timer plots](CControlledRateTimer_example.png)
 *
 * This graphs is generated with the example:
 *
 * `system_control_rate_timer_example --rate1 2000.0 --rate2 4000.0`
 *
 * All the parameters for the PI controller and low-pass filter (rate estimator)
 * are settable by the user to adapt them to specific needs.
 *
 * \note Control law by [Francisco Jose Mañas Alvarez](https://github.com/FranciscoJManasAlvarez)
 *
 * \note [New in MRPT 2.0.4]
 * \ingroup mrpt_system_grp
 */
class CControlledRateTimer : public mrpt::system::COutputLogger
{
   public:
	/** @name Main API
	 *  @{ */

	/** Ctor: specifies the desired rate (Hz) */
	CControlledRateTimer(const double rate_hz = 1.0);
	/** Dtor */
	virtual ~CControlledRateTimer() = default;

	/** Changes the object loop rate (Hz) */
	void setRate(const double rate_hz);

	/** Sleeps for some time, such as the return of this method is 1/rate
	 * (seconds)
	 * after the return of the previous call.
	 * \return false if the rate could not be achieved ("we are already late"),
	 * true if all went right. */
	bool sleep();

	/** @} */

	/** @name PI control parameters
	 *  @{ */

	/** PI controller Kp parameter [default=1.0] */
	double controllerParam_Kp() const { return m_Kp; }
	void controllerParam_Kp(double v)
	{
		ASSERT_ABOVE_(v, .0);
		m_Kp = v;
	}

	/** PI controller Ti parameter [default=0.0194] */
	double controllerParam_Ti() const { return m_Ti; }
	void controllerParam_Ti(double v)
	{
		ASSERT_ABOVEEQ_(v, .0);
		m_Ti = v;
	}

	/** Low-pass filter a0 value [default=0.9]:
	 * estimation = a0*input + (1-a0)*former_estimation */
	double lowPassParam_a0() const { return m_lowPass_a0; }
	void lowPassParam_a0(double v)
	{
		ASSERT_ABOVE_(v, .0);
		ASSERT_BELOWEQ_(v, 1.0);
		m_lowPass_a0 = v;
	}

	/** Get/set ratio threshold for issuing a warning (via COutputLogger
	 * interface) if the achieved rate is not this close to the set-point
	 * [Default=0.2, =20%]
	 */
	double followErrorRatioToRaiseWarning() const
	{
		return m_followErrorRatioForWarning;
	}
	void followErrorRatioToRaiseWarning(double v)
	{
		ASSERT_ABOVE_(v, .0);
		ASSERT_BELOWEQ_(v, 1.0);
		m_followErrorRatioForWarning = v;
	}

	/** Gets the actual controller output: the rate (Hz) of the internal
	 * CRateTimer object. */
	double actualControlledRate() const { return m_ratetimer.rate(); }

	/** Gets the latest estimated run rate (Hz), which comes from actual period
	 * measurement, low-pass filtered. */
	double estimatedRate() const { return m_currentEstimatedRate; }

	/** Last actual execution rate measured (Hz), without low-pass filtering */
	double estimatedRateRaw() const { return m_lastRawRate; }

	/** @} */
   private:
	double m_rate_hz = 1.0;
	mrpt::system::CRateTimer m_ratetimer;  //!< the one control acts on

	double m_lowPass_a0 = 0.9;
	double m_Kp = 1.0;
	double m_Ti = 0.0194;

	double m_followErrorRatioForWarning = 0.20;
	double m_lastControlError = .0;

	bool internalUpdateRateEstimate();
	double m_currentEstimatedRate = 1.0, m_lastRawRate = 1.0;
	double m_lastTic = 0;
	mrpt::system::CTicTac m_tic;

};  // End of class def.

}  // namespace mrpt::system
