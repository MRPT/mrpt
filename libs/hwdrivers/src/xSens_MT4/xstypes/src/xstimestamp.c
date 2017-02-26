/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#include "xstimestamp.h"
#include "xstime.h"
#include "xsutctime.h"
#include <time.h>

/*! \class XsTimeStamp
	\brief This class contains method to set, retrieve and compare timestamps
*/

/*! \addtogroup cinterface C Interface
	@{
*/

/*! \relates XsTimeStamp \brief Set the stored time to \a miliseconds. */
void XsTimeStamp_setMilliSecondTime(XsTimeStamp* thisPtr, int64_t t)
{
	thisPtr->m_msTime = t;
}

/*! \relates XsTimeStamp \brief Get the time of day component of the stored timestamp in seconds as a double precision value. */
double XsTimeStamp_timeOfDay(const XsTimeStamp* thisPtr)
{
	return (double)(thisPtr->m_msTime % (24*60*60*1000))*0.001;
}

/*! \relates XsTimeStamp \brief Returns the number of seconds elapsed since the epoch as stored in the XsTimeStamp
*/
int64_t XsTimeStamp_secondTime(const XsTimeStamp* thisPtr)
{
	return thisPtr->m_msTime/1000;
}

/*! \relates XsTimeStamp \brief Returns the millisecond part of the time (in the range 0-999)
*/
int32_t XsTimeStamp_milliSecondPart(const XsTimeStamp* thisPtr)
{
	return (int32_t) (thisPtr->m_msTime % 1000);
}

/*! \relates XsTimeStamp \brief Returns the seconds part of the time (in the range 0-59)
*/
int32_t XsTimeStamp_secondPart(const XsTimeStamp* thisPtr)
{
	return (int32_t) ((thisPtr->m_msTime/(1000))%60);
}

/*! \relates XsTimeStamp \brief Returns the minutes part of the time (in the range 0-59)
*/
int32_t XsTimeStamp_minutePart(const XsTimeStamp* thisPtr)
{
	return (int32_t) ((thisPtr->m_msTime/(60*1000))%60);
}

/*! \relates XsTimeStamp \brief Returns the hours part of the time (in the range 0-23)
*/
int32_t XsTimeStamp_hourPart(const XsTimeStamp* thisPtr)
{
	return (int32_t) ((thisPtr->m_msTime/(60*60*1000))%24);
}

/*! \relates XsTimeStamp
	\brief Returns the current time in ms since the epoch (Jan 1st 1970)
	\see XsTime_timeStampNow
	\param dest The object to write the time to, may be 0 in which case only the return value is generated.
	\returns The current time in ms since the epoch (Jan 1st 1970)
*/
int64_t XsTimeStamp_now(XsTimeStamp* dest)
{
	return (int64_t) XsTime_timeStampNow(dest);
}

/*! \relates XsTimeStamp
	\brief Returns the maximum value of an %XsTimeStamp
*/
int64_t XsTimeStamp_maxValue()
{
	return 9223372036854775807LL;	//INT64_MAX;
}

/*! \relates XsTimeStamp
	\brief Creates an XsTimeStamp from an XsUtcTime object
	\param utc The UTC time to convert
	\returns The converted time in ms since the epoch (Jan 1st 1970)
*/
int64_t XsTimeStamp_fromUtcTime(struct XsTimeStamp* thisPtr, const struct XsUtcTime* utc)
{
	int64_t rv = -1;
	if (utc && utc->m_valid)
	{
		struct tm utctm;
		utctm.tm_year = utc->m_year;
		utctm.tm_mon = utc->m_month;
		utctm.tm_mday = utc->m_day;

		utctm.tm_hour = utc->m_hour;
		utctm.tm_min = utc->m_minute;
		utctm.tm_sec = utc->m_second;

		// unused
		utctm.tm_wday = 0;
		utctm.tm_yday = 0;
		utctm.tm_isdst = 0;

		rv = (((int64_t) mktime(&utctm)) * 1000LL) + (utc->m_nano / 1000000);
	}
	if (thisPtr)
		thisPtr->m_msTime = rv;
	return rv;
}

/*! \relates XsTimeStamp
	\brief Converts the timestamp into an XsUtcTime object
	\param utc The UTC time object to write the conversion result to
*/
void XsTimeStamp_toUtcTime(struct XsTimeStamp* thisPtr, struct XsUtcTime* utc)
{	
	struct tm tmUtc;
#ifdef _WIN32		
	__time64_t t;
	t = (__time64_t)(thisPtr->m_msTime / 1000);
	if (_gmtime64_s(&tmUtc, &t))
	{
		//in case of an error the result is an invalid XsUtctime
		utc->m_valid = 0;
		return;
	}
#else
	time_t t;
	t = (time_t)(thisPtr->m_msTime / 1000);
	if (gmtime_r(&t, &tmUtc) == 0)
	{
		//in case of an error the result is an invalid XsUtctime
		utc->m_valid = 0;
		return;
	}
#endif

	utc->m_day = tmUtc.tm_mday + 1;
	utc->m_hour = tmUtc.tm_hour;
	utc->m_minute = tmUtc.tm_min;
	utc->m_month = tmUtc.tm_mon + 1;
	utc->m_nano = (uint32_t)((thisPtr->m_msTime % 1000) * 1e6);
	utc->m_second = tmUtc.tm_sec;
	utc->m_year = tmUtc.tm_year + 1900;
	utc->m_valid = 1;
}

/*! @} */
