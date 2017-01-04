/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef _XSENS_TIME_2006_09_12
#define _XSENS_TIME_2006_09_12

#include <mrpt/utils/mrpt_stdint.h>

#include <time.h>

namespace xsens {

//! The number of seconds in a normal day
#define XSENS_SEC_PER_DAY	(60*60*24)
//! The number of milliseconds in a normal day
#define XSENS_MS_PER_DAY	(XSENS_SEC_PER_DAY*1000)

//! A real-time timestamp (ms)
typedef uint64_t TimeStamp;

/*! \brief A platform-independent clock.

	The function returns the time of day in ms since midnight. If the \c date parameter is
	non-NULL, corresponding the date is placed in the variable it points to.
*/
uint32_t getTimeOfDay(tm* date_ = NULL, time_t* secs_ = NULL);

/*! \brief A platform-independent sleep routine.

	Time is measured in ms. The function will not return until the specified
	number of ms have passed.
*/
void msleep(uint32_t ms);

TimeStamp timeStampNow(void);

}	// end of xsens namespace

#endif	// _XSENS_TIME_2006_09_12
