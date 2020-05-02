
//  Copyright (c) 2003-2019 Xsens Technologies B.V. or subsidiaries worldwide.
//  All rights reserved.
//  
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//  
//  1.	Redistributions of source code must retain the above copyright notice,
//  	this list of conditions, and the following disclaimer.
//  
//  2.	Redistributions in binary form must reproduce the above copyright notice,
//  	this list of conditions, and the following disclaimer in the documentation
//  	and/or other materials provided with the distribution.
//  
//  3.	Neither the names of the copyright holders nor the names of their contributors
//  	may be used to endorse or promote products derived from this software without
//  	specific prior written permission.
//  
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
//  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
//  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
//  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
//  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS 
//  SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES 
//  OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE 
//  ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.
//  

#include "xstimestamp.h"
#include "xstime.h"
#include "xstimeinfo.h"
#ifndef XSENS_NO_STL
#include "xsstring.h"
#endif
#include <time.h>
#include <string.h>
#include <stdio.h>

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
int64_t XsTimeStamp_maxValue(void)
{
	return 9223372036854775807LL;	//INT64_MAX;
}

/*! \relates XsTimeStamp
	\brief Creates a (UTC) XsTimeStamp from an XsTimeInfo object
	\param info The time info object to convert
	\returns The converted UTC time in ms since the epoch (Jan 1st 1970)
*/
int64_t XsTimeStamp_fromTimeInfo(struct XsTimeStamp* thisPtr, const struct XsTimeInfo* info)
{
	int64_t rv = -1;
	int64_t epoch = 0;
	if (info && info->m_valid)
	{
		struct tm utctm;
		utctm.tm_year = info->m_year - 1900;
		utctm.tm_mon = info->m_month - 1;
		utctm.tm_mday = info->m_day;

		utctm.tm_hour = info->m_hour;
		utctm.tm_min = info->m_minute;
		utctm.tm_sec = info->m_second;

		// unused
		utctm.tm_wday = 0;
		utctm.tm_yday = 0;
		utctm.tm_isdst = 0;

#ifdef _WIN32
		epoch = (int64_t)_mkgmtime(&utctm);
#else
		epoch = (int64_t)timegm(&utctm);
#endif

		rv = (epoch * 1000LL) + (info->m_nano / 1000000) + ((int64_t) info->m_utcOffset * 60000);
	}
	if (thisPtr)
		thisPtr->m_msTime = rv;
	return rv;
}

/*! \relates XsTimeStamp
	\brief Converts the timestamp into an XsTimeInfo object
	\param info The XsTimeInfo time object to write the conversion result to
*/
void XsTimeStamp_toTimeInfo(struct XsTimeStamp const* thisPtr, struct XsTimeInfo* info)
{
	struct tm tmUtc;
#ifdef _WIN32
	__time64_t t;
	t = (__time64_t)(thisPtr->m_msTime / 1000);
	if (_gmtime64_s(&tmUtc, &t))
	{
		//in case of an error the result is an invalid XsTimeInfo
		info->m_valid = 0;
		return;
	}
#elif (defined(__arm__) && defined(__ARMCC_VERSION))
	#warning Function is not thread-safe for this platform/toolchain
	time_t t;
	t = (time_t)(thisPtr->m_msTime / 1000);
	struct tm* tmUtcPtr = gmtime(&t);
	if (tmUtcPtr == 0)
	{
		info->m_valid = 0;
		return;
	}
	memcpy(&tmUtc, tmUtcPtr, sizeof(tmUtc));
#else
	time_t t;
	t = (time_t)(thisPtr->m_msTime / 1000);
	if (gmtime_r(&t, &tmUtc) == 0)
	{
		//in case of an error the result is an invalid XsTimeInfo
		info->m_valid = 0;
		return;
	}
#endif

	info->m_day = (uint8_t) tmUtc.tm_mday;
	info->m_hour = (uint8_t) tmUtc.tm_hour;
	info->m_minute = (uint8_t) tmUtc.tm_min;
	info->m_month = (uint8_t) tmUtc.tm_mon + 1;
	info->m_nano = (uint32_t)((thisPtr->m_msTime % 1000) * 1e6);
	info->m_second = (uint8_t) tmUtc.tm_sec;
	info->m_year = (uint16_t) (tmUtc.tm_year + 1900);
	info->m_valid = 1;
	info->m_utcOffset = 0;
}

#ifndef XSENS_NO_STL
/*! \relates XsTimeStamp
	\brief Converts the timestamp into an XsString object in format YYYY/MM/DD hh:mm:ss.nnn
	\param result The XsString object to write the conversion result to
*/
void XsTimeStamp_toString(struct XsTimeStamp const* thisPtr, struct XsString* result)
{
	XsTimeInfo info;
	char buffer[32];
	XsTimeStamp_toTimeInfo(thisPtr, &info);

	if (info.m_valid)
	{
		sprintf(buffer, "%04d/%02d/%02d %02d:%02d:%02d.%03d"
			, (int) info.m_year
			, (int) info.m_month
			, (int) info.m_day
			, (int) info.m_hour
			, (int) info.m_minute
			, (int) info.m_second
			, (int) (thisPtr->m_msTime % 1000));
		XsString_assign(result, 23, buffer);
	}
	else
		XsString_assign(result, 23, "0000/00/00 00:00:00.000");
}
#endif

/*! \cond XS_INTERNAL */
extern int64_t XsTime_utcToLocalValue;	//!< Internal storage for UTC to local time correction (ms)
extern int64_t XsTime_localToUtcValue;	//!< Internal storage for local time to UTC correction (ms)
/*! \endcond */

/*! \brief Convert the supplied time from (assumed) UTC to local time, using the system's local time zone knowledge
	\param local The returned, converted time
	\note Make sure to call XsTime_initializeTime() before calling this function as well as when changing time zones.
*/
void XsTimeStamp_utcToLocalTime(struct XsTimeStamp const* thisPtr, struct XsTimeStamp* local)
{
	local->m_msTime = thisPtr->m_msTime + (thisPtr->m_msTime ? XsTime_utcToLocalValue : 0LL);
}

/*! \brief Convert the supplied time from (assumed) local time to UTC, using the system's local time zone knowledge
	\param utc The returned, converted time
	\note Make sure to call XsTime_initializeTime() before calling this function as well as when changing time zones.
*/
void XsTimeStamp_localToUtcTime(struct XsTimeStamp const* thisPtr, struct XsTimeStamp* utc)
{
	utc->m_msTime = thisPtr->m_msTime + (thisPtr->m_msTime ? XsTime_localToUtcValue : 0LL);
}

/*! \brief Convert the supplied time from (assumed) UTC to local time, using the offset in \a info
	\param local The returned, converted time
	\param info The XsTimeInfo object containing the time offset to apply
*/
void XsTimeStamp_utcToLocalTime2(struct XsTimeStamp const* thisPtr, struct XsTimeStamp* local, const struct XsTimeInfo* info)
{
	local->m_msTime = thisPtr->m_msTime - (thisPtr->m_msTime ? (int64_t) info->m_utcOffset * 60000 : 0LL);
}

/*! \brief Convert the supplied time from (assumed) local time to UTC, using the offset in \a info
	\param utc The returned, converted time
	\param info The XsTimeInfo object containing the time offset to apply
*/
void XsTimeStamp_localToUtcTime2(struct XsTimeStamp const* thisPtr, struct XsTimeStamp* utc, const struct XsTimeInfo* info)
{
	utc->m_msTime = thisPtr->m_msTime + (thisPtr->m_msTime ? (int64_t) info->m_utcOffset * 60000 : 0LL);
}

/*! \brief Convert the supplied time from (assumed) UTC to local time, using the offset in \a utcOffset
	\param local The returned, converted time
	\param utcOffset The time offset to apply as defined in XsTimeInfo::m_utcOffset but in ms instead of minutes
*/
void XsTimeStamp_utcToLocalTime_ms(struct XsTimeStamp const* thisPtr, struct XsTimeStamp* local, int64_t utcOffset)
{
	local->m_msTime = thisPtr->m_msTime - (thisPtr->m_msTime ? utcOffset: 0LL);
}

/*! \brief Convert the supplied time from (assumed) local time to UTC, using the offset in \a utcOffset
	\param utc The returned, converted time
	\param utcOffset The time offset to apply as defined in XsTimeInfo::m_utcOffset but in ms instead of minutes
*/
void XsTimeStamp_localToUtcTime_ms(struct XsTimeStamp const* thisPtr, struct XsTimeStamp* utc, int64_t utcOffset)
{
	utc->m_msTime = thisPtr->m_msTime + (thisPtr->m_msTime ? utcOffset : 0LL);
}

/*! @} */
