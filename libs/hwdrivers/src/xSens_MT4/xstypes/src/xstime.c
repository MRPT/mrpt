/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
// Note, this function requires compiler option "-lrt" to be set when compiling with gcc
#include "xstime.h"
#include "xstimestamp.h"
#include <stdio.h>
#include <wchar.h>

#ifdef _WIN32
	#include <windows.h>
	#define snprintf _snprintf
#else
	#include <unistd.h>
	#include <sys/time.h>
	#include <pthread.h>
	#include <string.h>
#endif
#include <time.h>

/*! \namespace XsTime
	\brief Namespace for time and date constants and operations
*/

//! The number of seconds in a normal day
const XsTimeStamp XsTime_secPerDay = { 60*60*24LL };

//! The number of milliseconds in a normal day
const XsTimeStamp XsTime_milliSecPerDay = { 60*60*24*1000LL };

//! The maximum positive value of an Xsens TimeStamp value
const XsTimeStamp XsTime_timeStampMax = { 0x7FFFFFFFFFFFFFFFLL };


//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// Other  functions ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
#ifdef __APPLE__
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
	static const int64_t CORRECTION_DELTA_MS = 32; //! \todo This value was experimentally determined. 20 is too low. Find better solution
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
			QueryPerformanceFrequency(&tmp);	//lint !e534 the return value should match that of QueryPerformanceCounter above
			perfCountFreq = tmp.QuadPart;
			startTimePerfCount = pc.QuadPart;
			startTimeSysTime = t;
		}

		tNow = startTimeSysTime + (1000*(pc.QuadPart - startTimePerfCount))/perfCountFreq;

		if (t > tNow || (tNow-t) > CORRECTION_DELTA_MS) //!< \todo find better solution
		{
			startTimePerfCount = pc.QuadPart;
			startTimeSysTime = t;
		}
		else
			t = tNow;
	}

	tin = t/1000;
	if (date_ != NULL)
		_localtime64_s(date_,&tin);	//lint !e534
	if (secs_ != NULL)
		*secs_ = (time_t) tin;
	return (uint32_t) (t % (XsTime_secPerDay.m_msTime*1000)); //lint !e571 cast is ok
#else
	struct timespec tp;
	clock_gettime(CLOCK_REALTIME, &tp); // compile with -lrt

	if (date_ != NULL)
		localtime_r(&tp.tv_sec,date_);

	if (secs_ != NULL)
		secs_[0] = tp.tv_sec;

	// 86400 = 24*60*60 = secs in a day, this gives us the seconds since midnight
	return (1000 * (tp.tv_sec % XsTime_secPerDay.m_msTime)) + (tp.tv_nsec/1000000);
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
	_time64(&t);	//lint !e534 return value is stored in t
	if (date != 0)
		_localtime64_s(date, &t);	//lint !e534
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
		XsTime_getDateTime(&dt);	//lint !e534 return value is stored in dt

	year = dt.tm_year + 1900;
	month = dt.tm_mon + 1;
	snprintf(dest, 8, "%04d%02d%02d", year, month, dt.tm_mday);		//lint !e534
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
		XsTime_getDateTime(&dt);	//lint !e534 return value is stored in dt

	snprintf(dest, 8, "%02d%02d%02d%02d", dt.tm_hour, dt.tm_min, dt.tm_sec, 0);	//lint !e534
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
		XsTime_getDateTime(&dt);	//lint !e534 return value is stored in dt

	year = dt.tm_year + 1900;
	month = dt.tm_mon + 1;
	swprintf(dest, 8, L"%04d%02d%02d", year, month, dt.tm_mday);	//lint !e534
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
		XsTime_getDateTime(&dt);	//lint !e534 return value is stored in dt

	swprintf(dest, 8, L"%02d%02d%02d%02d", dt.tm_hour, dt.tm_min, dt.tm_sec, 0);	//lint !e534
}

/*! \brief Make the current thread sleep for at least \a ms milliseconds
	\details A platform independent sleep routine to sleep for at least \a ms milliseconds.

	On linux we are using pthread_cond_t, which implies that the timedwait() is a cancellation point.
	This function might return sooner than expected in some cases due to system events.

	\param ms The number of milliseconds to sleep
*/
void XsTime_msleep(uint32_t ms)
{
#ifdef _WIN32
	Sleep(ms);
#elif defined(__APPLE__)
	int64_t diff;
	clock_t end = clock() + (CLOCKS_PER_SEC/1000) * ms;

	while ((diff = end - clock()) > 0)
	{
		diff = (1000 * diff) / CLOCKS_PER_SEC;
		if (diff > 1000)
			sleep(diff / 1000);
		else
			usleep(diff * 1000);
	}
#else
	int64_t s, ns;
	pthread_cond_t c;
	pthread_condattr_t ca;
	pthread_mutex_t m;
	struct timespec ts;
	int ret;
	clockid_t clockid = CLOCK_MONOTONIC;

	pthread_condattr_init(&ca);

	pthread_mutex_init(&m, NULL);

	// using the monotonic clock -- The real time clock is affected by external processes such as ntp and influence the delay too much.
	ret = pthread_condattr_setclock(&ca, clockid);
	// if this one failed (which should not happen), get the currently used clock instead
	if (ret)
		pthread_condattr_getclock(&ca, &clockid);

	pthread_cond_init(&c, &ca);

	clock_gettime(clockid, &ts);

	s = ms/1000;
	ts.tv_sec += s;

	ms -= s*1000;
	ns = ts.tv_nsec + (ms * 1000000);

	ts.tv_sec += ns / 1000000000L;
	ts.tv_nsec = ns % 1000000000L;

	pthread_mutex_lock(&m);
	pthread_cond_timedwait(&c, &m, &ts);

	pthread_mutex_unlock(&m);
	pthread_cond_destroy(&c);
	pthread_condattr_destroy(&ca);
	pthread_mutex_destroy(&m);
#endif
}

/*! \brief Delays the current thread for at least \a us microseconds
	\details A platform independent delay routine to sleep for at least \a us microseconds.

	On linux we are using pthread_cond_t, which implies that the timedwait() is a cancellation point.
	This function might return sooner than expected in some cases due to system events.

	\param us The number of microseconds to delay
*/
void XsTime_udelay(uint32_t us)
{
#ifdef _WIN32
	//lint --e{534} Ignoring return value of query functions.
	LARGE_INTEGER freq, start, stop;
	double countPerMicroSecond ;
	QueryPerformanceCounter(&start);
	QueryPerformanceFrequency(&freq);
	countPerMicroSecond = freq.QuadPart / 1.0e6;
	do {
		QueryPerformanceCounter(&stop);
	}
	while (start.QuadPart + ((double)(us) * countPerMicroSecond) > stop.QuadPart);

#elif defined(__APPLE__)
	usleep(us);
#else
	int64_t s, ns;
	pthread_cond_t c;
	pthread_condattr_t ca;
	pthread_mutex_t m;
	struct timespec ts;
	int ret;
	clockid_t clockid = CLOCK_MONOTONIC;

	pthread_condattr_init(&ca);

	pthread_mutex_init(&m, NULL);

	// using the monotonic clock -- The real time clock is affected by external processes such as ntp and influence the delay too much.
	ret = pthread_condattr_setclock(&ca, clockid);
	// if this one failed (which should not happen), get the currently used clock instead
	if (ret)
		pthread_condattr_getclock(&ca, &clockid);

	pthread_cond_init(&c, &ca);

	clock_gettime(clockid, &ts);

	s = us/1e6;
	ts.tv_sec += s;

	ns = ts.tv_nsec + (us * 1000);

	ts.tv_sec += ns / 1000000000L;
	ts.tv_nsec = ns % 1000000000L;

	pthread_mutex_lock(&m);
	pthread_cond_timedwait(&c, &m, &ts);

	pthread_mutex_unlock(&m);
	pthread_cond_destroy(&c);
	pthread_condattr_destroy(&ca);
	pthread_mutex_destroy(&m);
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

/*! \brief Stabilize the clock
	\details Repeatedly call XsTime_timeStampNow for 16-32 ms to stabilize the clock
*/
void XsTime_initializeTime()
{
	long long start = XsTime_timeStampNow(0);
	while (XsTime_timeStampNow(0) - start < 32) {}
}

/*! @} */
