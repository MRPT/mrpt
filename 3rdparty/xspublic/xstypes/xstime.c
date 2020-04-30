
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

// Note, this function requires compiler option "-lrt" to be set when compiling with gcc
#include "xstime.h"
#include "xstimestamp.h"
#include <stdio.h>
#include <wchar.h>

#ifdef _WIN32
#	include <windows.h>
#	define snprintf _snprintf
#else
#	include <errno.h>
#	include <unistd.h>
#	include <sys/time.h>
#	include <pthread.h>
#	include <string.h>
#endif
#include <time.h>

/*! \namespace XsTime
	\brief Namespace for time and date constants and operations
*/

//! The number of seconds in a normal day
const XsTimeStamp XsTime_secPerDay = { 60*60*24LL };

//! The number of milliseconds in a normal day
const XsTimeStamp XsTime_milliSecPerDay = { 60*60*24*1000LL };

//! The maximum positive value of an XsTimeStamp value
const XsTimeStamp XsTime_timeStampMax = { 0x7FFFFFFFFFFFFFFFLL };

//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// Other  functions ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
#if 0 // def __APPLE__, it is in fact available on apple, so...
/*! \brief An implementation of linux' clock_gettime()

	clock_gettime() is not available on Apple/Darwin platforms. This function helps
	maintaining compatibility with Linux code.
 */
#ifndef CLOCK_REALTIME
#define CLOCK_REALTIME 0
#endif
static int clock_gettime(int clk_id, struct timespec *tp)
{
	(void)clk_id;
	struct timeval now;

	int rv = gettimeofday(&now, NULL);
	if (rv != 0)
		return rv;

	tp->tv_sec = now.tv_sec;
	tp->tv_nsec = now.tv_usec * 1000;

	return 0;
}
#endif

/*! \addtogroup cinterface C Interface
	@{
*/

//////////////////////////////////////////////////////////////////////////////////////////
/*! \brief The function returns the current time of day in ms since midnight
	\param date_ When not 0, the corresponding current date is put in here.
	\param secs_ When not 0, the full timestamp in seconds since the epoch is put in here.
	\returns The number of milliseconds that have passed since midnight.
*/
uint32_t XsTime_getTimeOfDay(struct tm* date_, time_t* secs_)
{
#ifdef _WIN32
	// CORRECTION_DELTA_MS is the maximum allowed difference between the system time and the performance counter time. This depends on the granularity of the clocks.
	static const int64_t CORRECTION_DELTA_MS = 32;
	static int64_t startTimePerfCount;
	static int64_t startTimeSysTime;
	static int64_t perfCountFreq = 0;
	int64_t t;
	LARGE_INTEGER pc;
	FILETIME now;
	__time64_t tin;

	GetSystemTimeAsFileTime(&now);
	t = (int64_t) (((((uint64_t) now.dwHighDateTime << 32) | now.dwLowDateTime)/10000) - 11644473600000);

	if (QueryPerformanceCounter(&pc))
	{
		int64_t tNow;
		if (!perfCountFreq)
		{
			LARGE_INTEGER tmp;
			QueryPerformanceFrequency(&tmp);
			perfCountFreq = tmp.QuadPart;
			startTimePerfCount = pc.QuadPart;
			startTimeSysTime = t;
		}

		tNow = startTimeSysTime + (1000*(pc.QuadPart - startTimePerfCount))/perfCountFreq;

		if (t > tNow || (tNow-t) > CORRECTION_DELTA_MS)
		{
			startTimePerfCount = pc.QuadPart;
			startTimeSysTime = t;
		}
		else
			t = tNow;
	}

	tin = t/1000;
	if (date_ != NULL)
		_localtime64_s(date_,&tin);
	if (secs_ != NULL)
		*secs_ = (time_t) tin;
	return (uint32_t) (t % (XsTime_secPerDay.m_msTime*1000));
#else
	struct timespec tp;
	clock_gettime(CLOCK_REALTIME, &tp); // compile with -lrt

	if (date_ != NULL)
		localtime_r(&tp.tv_sec,date_);

	if (secs_ != NULL)
		secs_[0] = tp.tv_sec;

	// 86400 = 24*60*60 = secs in a day, this gives us the seconds since midnight
	return (uint32_t)((1000 * (tp.tv_sec % XsTime_secPerDay.m_msTime)) + (tp.tv_nsec/1000000));
#endif
}

/*! \brief Retrieves the date and time (platform-independent)
	\param date : if non-zero the local (!) date and time is stored in the tm struct this parameter points to
	\returns The UTC date and time as seconds since 1970
*/
int64_t XsTime_getDateTime(struct tm *date)
{
#ifdef _WIN32
	__time64_t t;
	_time64(&t);
	if (date != 0)
		_localtime64_s(date, &t);
	return (int64_t)t;
#else
	time_t t;
	time(&t);
	if(date != 0)
	{
		struct tm *result = localtime(&t);
		memcpy(date, result, sizeof(struct tm));
	}
	return (int64_t)t;
#endif
}

/*! \brief Retrieves the date as string representation
	The format is YYYYMMDD
	so 25 dec 2010 is stored as an array dest[8] = {'2', '0', '1', '0', '1', '2', '2', '5' }
	\param dest : A pointer to an array of at least (!) 8 bytes
	\param date : If date is non-zero this date is converted, otherwise the current date is retrieved and used)
*/
void XsTime_getDateAsString(char* dest, const struct tm* date)
{
	struct tm dt;
	int year, month;

	if(date != 0)
		dt = *date;
	else
		XsTime_getDateTime(&dt);

	year = dt.tm_year + 1900;
	month = dt.tm_mon + 1;
	snprintf(dest, 9, "%04d%02d%02d", year, month, dt.tm_mday);
}

/*! \brief Retrieves the time as binary
	The format is HHMMSShh (where H is hour and 'h' is hundredths)
	so 14:25:01.23 is stored as an array dest[8] = { '1', '4', '2', '5', '0',  '1', '2', '3'}
	\param dest : A pointer to an array of at least (!) 8 bytes
	\param date : If date is non-zero this date is converted, otherwise the current date is retrieved and used)
	\note (for now hundreths are set to 0)
*/
void XsTime_getTimeAsString(char* dest, const struct tm* date)
{
	struct tm dt;

	if (date != 0)
		dt = *date;
	else
		XsTime_getDateTime(&dt);

	snprintf(dest, 8, "%02d%02d%02d%02d", dt.tm_hour, dt.tm_min, dt.tm_sec, 0);
}

/*! \brief Retrieves the date as wstring representation
	The format is YYYYMMDD
	so 25 dec 2010 is stored as an array dest[8] = {'2', '0', '1', '0', '1', '2', '2', '5' }
	\param dest : A pointer to an array of at least (!) 8 wchars
	\param date : If date is non-zero this date is converted, otherwise the current date is retrieved and used)
*/
void XsTime_getDateAsWString(wchar_t* dest, const struct tm* date)
{
	struct tm dt;
	int year, month;

	if(date != 0)
		dt = *date;
	else
		XsTime_getDateTime(&dt);

	year = dt.tm_year + 1900;
	month = dt.tm_mon + 1;
	swprintf(dest, 9, L"%04d%02d%02d", year, month, dt.tm_mday);
}

/*! \brief Retrieves the time as binary
	The format is HHMMSShh (where H is hour and 'h' is hundredths)
	so 14:25:01.23 is stored as an array dest[8] = { '1', '4', '2', '5', '0',  '1', '2', '3'}
	\param dest : A pointer to an array of at least (!) 8 wchars
	\param date : If date is non-zero this date is converted, otherwise the current date is retrieved and used)
	\note (for now hundreths are set to 0)
*/
void XsTime_getTimeAsWString(wchar_t* dest, const struct tm* date)
{
	struct tm dt;

	if (date != 0)
		dt = *date;
	else
		XsTime_getDateTime(&dt);

	swprintf(dest, 9, L"%02d%02d%02d%02d", dt.tm_hour, dt.tm_min, dt.tm_sec, 0);
}

/*! \brief Make the current thread sleep for at least \a ms milliseconds
	\details A platform independent sleep routine to sleep for at least \a ms milliseconds.

	\param ms The number of milliseconds to sleep

	\sa XsTime_udelay
*/
void XsTime_msleep(uint32_t ms)
{
#ifdef _WIN32
	Sleep(ms);
#else
	XsTime_udelay(ms * 1000);
#endif
}

/*! \brief Delays the current thread for at least \a us microseconds
	\details A platform independent delay routine to sleep for at least \a us microseconds.

	\param us The number of microseconds to delay

	\sa XsTime_msleep
*/
void XsTime_udelay(uint64_t us)
{
#ifdef _WIN32
	LARGE_INTEGER freq, start, stop;
	double countPerMicroSecond ;
	QueryPerformanceCounter(&start);
	QueryPerformanceFrequency(&freq);
	countPerMicroSecond = freq.QuadPart / 1.0e6;
	do
	{
		QueryPerformanceCounter(&stop);
	}
	while (start.QuadPart + ((double)(us) * countPerMicroSecond) > stop.QuadPart);
#else
	struct timespec ts;
	int ret = -1;

	ts.tv_sec = us / 1000000;
	ts.tv_nsec = (us - (ts.tv_sec * 1000000)) * 1000;

	while (ret)
	{
		ret = nanosleep(&ts, &ts);
		if (ret)
			assert(errno == EINTR);
	}
#endif
}

/*! \brief Returns the current time in ms since the epoch (Jan 1st 1970)
	\param now Pointer to %XsTimeStamp container for the returned value, may be 0
	\returns The current time in ms since the epoch (Jan 1st 1970) as a 64-bit integer
*/
int64_t XsTime_timeStampNow(XsTimeStamp* now)
{
	XsTimeStamp tmp;
	time_t s;

	if (now == 0)
		now = &tmp;

	now->m_msTime = (long long) XsTime_getTimeOfDay(NULL,&s);
	now->m_msTime = (now->m_msTime % 1000) + (((long long)s)*1000);

	return now->m_msTime;
}

/*! \cond XS_INTERNAL */
int64_t XsTime_utcToLocalValue = 0;	//!< Internal storage for UTC to local time correction (ms)
int64_t XsTime_localToUtcValue = 0;	//!< Internal storage for local time to UTC correction (ms)
/*! \endcond XS_INTERNAL */

/*! \brief Stabilize the clock
	\details Repeatedly calls XsTime_timeStampNow for 16-32 ms to stabilize the clock.
	Initializes the local to UTC conversion parameters.
*/
void XsTime_initializeTime()
{
	int64_t tutc, tloc;
#ifdef _WIN32
	FILETIME now, nowloc;

	GetSystemTimeAsFileTime(&now);
	FileTimeToLocalFileTime(&now, &nowloc);

	tutc = (int64_t) (((((uint64_t) now.dwHighDateTime << 32) | now.dwLowDateTime)/10000) - 11644473600000);
	tloc = (int64_t) (((((uint64_t) nowloc.dwHighDateTime << 32) | nowloc.dwLowDateTime)/10000) - 11644473600000);

#else
	struct timespec tp;
	struct tm now, nowloc;

	clock_gettime(CLOCK_REALTIME, &tp); // compile with -lrt
	gmtime_r(&tp.tv_sec, &now);
	localtime_r(&tp.tv_sec, &nowloc);

	tutc = (int64_t) now.tm_min * 60000;
	tloc = (int64_t) nowloc.tm_min * 60000;
#endif

	XsTime_utcToLocalValue = tloc - tutc;
	XsTime_localToUtcValue = tutc - tloc;

	long long start = XsTime_timeStampNow(0);
	while (XsTime_timeStampNow(0) - start < 32) {}
}

/*! \brief Returns the conversion value from UTC time to local time in ms
	\returns The conversion value from UTC time to local time in ms
	\note Make sure to call XsTime_initializeTime() before calling this function as well as when changing time zones.
*/
int64_t XsTime_utcToLocal()
{
	return XsTime_utcToLocalValue;
}

/*! \brief Returns the conversion value from local time to UTC time in ms
	\returns The conversion value from local time to UTC time in ms
	\note Make sure to call XsTime_initializeTime() before calling this function as well as when changing time zones.
*/
int64_t XsTime_localToUtc()
{
	return XsTime_localToUtcValue;
}

/*! @} */
