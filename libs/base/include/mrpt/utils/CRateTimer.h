/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/utils/CTicTac.h>
#include <mrpt/base/link_pragmas.h>

namespace mrpt
{
namespace utils
{
	/** A class for calling sleep() in a loop, such that the amount of sleep time will be computed 
	 *  to make the loop run at the desired rate (in Hz).
	 * \note [New in MRPT 1.5.0]
	 * \ingroup mrpt_base_grp
	 */
	class BASE_IMPEXP CRateTimer
	{
	public:
		CRateTimer(const double rate_hz=1.0); //!< Ctor: specifies the desired rate (Hz)
		virtual ~CRateTimer();  //!< Dtor

		void setRate(const double rate_hz); //!< Changes the object loop rate (Hz)
		/** Sleeps for some time, such as the return of this method is 1/rate (seconds) 
		  * after the return of the previous call.
		  * \return false if the rate could not be achieved ("we are already late"), true if all went right. */
		bool sleep(); 
	private:
		double m_rate_hz;
		mrpt::utils::CTicTac m_tictac;
	}; // End of class def.

} // End of namespace
} // End of namespace
