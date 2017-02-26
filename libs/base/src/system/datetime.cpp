/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers

#include <mrpt/utils/utils_defs.h>
#include <mrpt/system/datetime.h>
#include <mrpt/system/os.h>

#ifdef MRPT_OS_WINDOWS
	#include <conio.h>
	#include <windows.h>
	#include <process.h>
	#include <tlhelp32.h>
	#include <sys/utime.h>
	#include <io.h>
	#include <direct.h>
#else
    #include <pthread.h>
    #include <termios.h>
    #include <unistd.h>
    #include <sys/select.h>
    #include <sys/time.h>
	#include <unistd.h>
	#include <utime.h>
	#include <errno.h>
#endif

#include <time.h>
#include <sys/types.h>
#include <sys/stat.h>

using namespace mrpt;
using namespace mrpt::system;
using namespace std;

mrpt::system::TTimeStamp  mrpt::system::time_tToTimestamp(const time_t &t )
{
	return uint64_t(t) * UINT64_C(10000000) +  UINT64_C(116444736) * UINT64_C(1000000000);
}

mrpt::system::TTimeStamp  mrpt::system::time_tToTimestamp(const double t )
{
	return uint64_t(t*10000000.0)+ UINT64_C(116444736)*UINT64_C(1000000000);
}

double mrpt::system::timestampTotime_t( const mrpt::system::TTimeStamp t )
{
	return double(t - UINT64_C(116444736)*UINT64_C(1000000000)) / 10000000.0;
}

/* Jerome Monceaux 2011/03/08: bilock@gmail.com
 * comment this include because it is not find
 * under snow leopard
 */
#if defined(MRPT_OS_APPLE)
//#	include <CFBase.h> // for CFAbsoluteTimeGetCurrent
# include <sys/timeb.h>
# include <sys/types.h>
#endif

/*---------------------------------------------------------------
					Returns the current system time.
  ---------------------------------------------------------------*/
mrpt::system::TTimeStamp  mrpt::system::getCurrentTime( )
{
#ifdef MRPT_OS_WINDOWS
	FILETIME		t;
	GetSystemTimeAsFileTime(&t);
	return (((uint64_t)t.dwHighDateTime) << 32) | ((uint64_t)t.dwLowDateTime);
#elif defined(MRPT_OS_APPLE)

	/* Jerome Monceaux 2011/03/08: bilock@gmail.com
	 * comment the next line because it does not compile
	 * under snow osx and an exception was thrown systematically
	 */
	struct timeval tv;
	timespec  tim;

	gettimeofday(&tv, NULL);
	tim.tv_sec = tv.tv_sec;
	tim.tv_nsec = tv.tv_usec*1000;

	return time_tToTimestamp( tim.tv_sec ) + tim.tv_nsec/100;
#else
    timespec  tim;
    clock_gettime(CLOCK_REALTIME, &tim);
	return time_tToTimestamp( tim.tv_sec ) + tim.tv_nsec/100;
#endif
}

/*---------------------------------------------------------------
					timestampToParts
  ---------------------------------------------------------------*/
void mrpt::system::timestampToParts( TTimeStamp t, TTimeParts &p , bool localTime)
{
	const double T = mrpt::system::timestampTotime_t(t);
	double sec_frac = T - floor(T);
	ASSERT_(sec_frac<1.0);

	const time_t tt = time_t(T);

	struct tm * parts =  localTime ? localtime(&tt) : gmtime(&tt);
	ASSERTMSG_(parts, "Malformed timestamp");

	p.year		= parts->tm_year + 1900;
	p.month		= parts->tm_mon + 1;
	p.day		= parts->tm_mday;
	p.day_of_week = parts->tm_wday + 1;
	p.daylight_saving = parts->tm_isdst;
	p.hour		= parts->tm_hour;
	p.minute	= parts->tm_min;
	p.second	= parts->tm_sec + sec_frac;
}



/*---------------------------------------------------------------
					buildTimestampFromParts
  ---------------------------------------------------------------*/
TTimeStamp mrpt::system::buildTimestampFromParts( const TTimeParts &p )
{
	struct tm parts;

	parts.tm_year  = p.year - 1900;
	parts.tm_mon   = p.month-1;
	parts.tm_mday  = p.day;
	parts.tm_wday  = p.day_of_week - 1;
	parts.tm_isdst = p.daylight_saving;
	parts.tm_hour  = p.hour;
	parts.tm_min   = p.minute;
	parts.tm_sec   = int(p.second);

	double sec_frac = p.second - parts.tm_sec;

	time_t  tt = mrpt::system::os::timegm(&parts); // Local time: mktime

	return mrpt::system::time_tToTimestamp( double(tt) + sec_frac );
}

/*---------------------------------------------------------------
					buildTimestampFromPartsLocalTime
  ---------------------------------------------------------------*/
TTimeStamp mrpt::system::buildTimestampFromPartsLocalTime( const TTimeParts &p )
{
	struct tm parts;

	parts.tm_year  = p.year - 1900;
	parts.tm_mon   = p.month-1;
	parts.tm_mday  = p.day;
	parts.tm_wday  = p.day_of_week - 1;
	parts.tm_isdst = p.daylight_saving;
	parts.tm_hour  = p.hour;
	parts.tm_min   = p.minute;
	parts.tm_sec   = int(p.second);

	double sec_frac = p.second - parts.tm_sec;

	time_t  tt = mktime(&parts);

	return mrpt::system::time_tToTimestamp( double(tt) + sec_frac );
}

/*---------------------------------------------------------------
					Returns the current local time.
  ---------------------------------------------------------------*/
mrpt::system::TTimeStamp  mrpt::system::getCurrentLocalTime()
{
#ifdef MRPT_OS_WINDOWS
	FILETIME		tt,t;
	GetSystemTimeAsFileTime(&tt);
	FileTimeToLocalFileTime(&tt,&t);

	return (((uint64_t)t.dwHighDateTime) << 32) | ((uint64_t)t.dwLowDateTime);
#elif defined(MRPT_OS_APPLE)
    // See: http://www.wand.net.nz/~smr26/wordpress/2009/01/19/monotonic-time-in-mac-os-x/
    THROW_EXCEPTION("to do")
#else
    timespec  tim;
    clock_gettime(CLOCK_REALTIME, &tim);

	time_t  tt;
	struct tm * timeinfo;
	time(&tt);
	timeinfo = localtime( &tt );

    return time_tToTimestamp( mktime(timeinfo) ) + tim.tv_nsec/100;
#endif
}

mrpt::system::TTimeStamp mrpt::system::timestampAdd( const mrpt::system::TTimeStamp tim, const double num_seconds)
{
	return static_cast<mrpt::system::TTimeStamp>(tim + static_cast<int64_t>(num_seconds*10000000.0));
}

/*---------------------------------------------------------------
					timeDifference
  ---------------------------------------------------------------*/
double mrpt::system::timeDifference( const mrpt::system::TTimeStamp t1, const mrpt::system::TTimeStamp t2 )
{
	MRPT_START
	ASSERT_(t1!=INVALID_TIMESTAMP)
	ASSERT_(t2!=INVALID_TIMESTAMP)

	return (int64_t(t2)- int64_t(t1))/10000000.0;

	MRPT_END
}

/*---------------------------------------------------------------
					secondsToTimestamp
  ---------------------------------------------------------------*/
mrpt::system::TTimeStamp mrpt::system::secondsToTimestamp( const double nSeconds )
{
	return (mrpt::system::TTimeStamp)(nSeconds*10000000.0);
}

/*---------------------------------------------------------------
					formatTimeInterval
  ---------------------------------------------------------------*/
string mrpt::system::formatTimeInterval( const double t )
{
	double timeSeconds = (t<0) ? (-t) : t;

	unsigned int nHours =  (unsigned int)timeSeconds / 3600;
	unsigned int nMins  = ((unsigned int)timeSeconds % 3600) / 60 ;
	unsigned int nSecs  = (unsigned int)timeSeconds % 60;
	unsigned int milSecs= (unsigned int) ( 1000*(timeSeconds - floor(timeSeconds)) );

	return format(
		"%02u:%02u:%02u.%03u",
		nHours,
		nMins,
		nSecs,
		milSecs );
}

/*---------------------------------------------------------------
  Convert a timestamp into this textual form: YEAR/MONTH/DAY,HH:MM:SS.MMM
  ---------------------------------------------------------------*/
string  mrpt::system::dateTimeToString(const mrpt::system::TTimeStamp t)
{
	if (t==INVALID_TIMESTAMP) return string("INVALID_TIMESTAMP");

	uint64_t        tmp = (t - ((uint64_t)116444736*1000000000));
    time_t          auxTime = tmp / (uint64_t)10000000;
    unsigned int	secFractions = (unsigned int)( 1000000 * (tmp % 10000000) / 10000000.0 );
    tm  *ptm = gmtime( &auxTime );

	if (!ptm)
		return std::string("(Malformed timestamp)");

	return format(
		"%u/%02u/%02u,%02u:%02u:%02u.%06u",
		1900+ptm->tm_year,
		ptm->tm_mon+1,
		ptm->tm_mday,
		ptm->tm_hour,
		ptm->tm_min,
		(unsigned int)ptm->tm_sec,
		secFractions );
}

/*---------------------------------------------------------------
  Convert a timestamp into this textual form (in local time):
      YEAR/MONTH/DAY,HH:MM:SS.MMM
  ---------------------------------------------------------------*/
string  mrpt::system::dateTimeLocalToString(const mrpt::system::TTimeStamp t)
{
	if (t==INVALID_TIMESTAMP) return string("INVALID_TIMESTAMP");

	uint64_t        tmp = (t - ((uint64_t)116444736*1000000000));
    time_t          auxTime = tmp / (uint64_t)10000000;
    unsigned int	secFractions = (unsigned int)( 1000000 * (tmp % 10000000) / 10000000.0 );
    tm  *ptm = localtime( &auxTime );

	if (!ptm) return "(Malformed timestamp)";

	return format(
		"%u/%02u/%02u,%02u:%02u:%02u.%06u",
		1900+ptm->tm_year,
		ptm->tm_mon+1,
		ptm->tm_mday,
		ptm->tm_hour,
		ptm->tm_min,
		(unsigned int)ptm->tm_sec,
		secFractions );
}

/*---------------------------------------------------------------
						extractDayTimeFromTimestamp
  ---------------------------------------------------------------*/
double  mrpt::system::extractDayTimeFromTimestamp(const mrpt::system::TTimeStamp t)
{
	MRPT_START
	ASSERT_(t!=INVALID_TIMESTAMP)

#ifdef MRPT_OS_WINDOWS
	SYSTEMTIME		sysT;
	FileTimeToSystemTime( (FILETIME*)&t, &sysT );
	return sysT.wHour * 3600.0 + sysT.wMinute * 60.0 + sysT.wSecond + sysT.wMilliseconds * 0.001;
#else
    time_t      auxTime = (t - ((uint64_t)116444736*1000000000)) / (uint64_t)10000000;
    tm  *ptm = gmtime( &auxTime );
	ASSERTMSG_(ptm, "Malformed timestamp");
	return ptm->tm_hour * 3600.0 + ptm->tm_min * 60.0 + ptm->tm_sec;
#endif
	MRPT_END
}


/*---------------------------------------------------------------
  Convert a timestamp into this textual form: HH:MM:SS.MMM
  ---------------------------------------------------------------*/
string  mrpt::system::timeLocalToString(const mrpt::system::TTimeStamp t, unsigned int secondFractionDigits)
{
	if (t==INVALID_TIMESTAMP) return string("INVALID_TIMESTAMP");

	uint64_t        tmp = (t - ((uint64_t)116444736*1000000000));
	const time_t          auxTime = tmp / (uint64_t)10000000;
	const tm  *ptm = localtime( &auxTime );

	unsigned int	secFractions = (unsigned int)( 1000000 * (tmp % 10000000) / 10000000.0 );
	// We start with 10^{-6} second units: reduce if requested by user:
	const unsigned int user_secondFractionDigits = secondFractionDigits;
	while (secondFractionDigits++<6)
		secFractions = secFractions / 10;

	return format(
		"%02u:%02u:%02u.%0*u",
		ptm->tm_hour,
		ptm->tm_min,
		(unsigned int)ptm->tm_sec,
		user_secondFractionDigits,
		secFractions );
}

/*---------------------------------------------------------------
  Convert a timestamp into this textual form: HH:MM:SS.MMM
  ---------------------------------------------------------------*/
string  mrpt::system::timeToString(const mrpt::system::TTimeStamp t)
{
	if (t==INVALID_TIMESTAMP) return string("INVALID_TIMESTAMP");

	uint64_t        tmp = (t - ((uint64_t)116444736*1000000000));
    time_t          auxTime = tmp / (uint64_t)10000000;
    unsigned int	secFractions = (unsigned int)( 1000000 * (tmp % 10000000) / 10000000.0 );
    tm  *ptm = gmtime( &auxTime );
	if (!ptm)
		return string("(Malformed timestamp)");

	return format(
		"%02u:%02u:%02u.%06u",
		ptm->tm_hour,
		ptm->tm_min,
		(unsigned int)ptm->tm_sec,
		secFractions );
}

/*---------------------------------------------------------------
  Convert a timestamp into this textual form: YEAR/MONTH/DAY
  ---------------------------------------------------------------*/
string  mrpt::system::dateToString(const mrpt::system::TTimeStamp t)
{
	if (t==INVALID_TIMESTAMP) return string("INVALID_TIMESTAMP");

	uint64_t        tmp = (t - ((uint64_t)116444736*1000000000));
    time_t          auxTime = tmp / (uint64_t)10000000;
    tm  *ptm = gmtime( &auxTime );
	if (!ptm)
		return string("(Malformed timestamp)");

	return format(
		"%u/%02u/%02u",
		1900+ptm->tm_year,
		ptm->tm_mon+1,
		ptm->tm_mday
		);
}

/** This function implements time interval formatting: Given a time in seconds, it will return a string describing the interval with the most appropriate unit.
 * E.g.: 1.23 year, 3.50 days, 9.3 hours, 5.3 minutes, 3.34 sec, 178.1 ms,  87.1 us.
 */
std::string
mrpt::system::intervalFormat(const double seconds)
{
	if (seconds>=365*24*3600)
			return format("%.2f years",seconds/(365*24*3600) );
	else if (seconds>=24*3600)
			return format("%.2f days",seconds/(24*3600));
	else if (seconds>=3600)
			return format("%.2f hours",seconds/3600);
	else if (seconds>=60)
			return format("%.2f minutes",seconds/60);
	else if (seconds>=1)
			return format("%.2f sec",seconds);
	else if (seconds>=1e-3)
			return format("%.2f ms",seconds*1e3);
	else if (seconds>=1e-6)
			return format("%.2f us",seconds*1e6);
	else	return format("%.2f ns",seconds*1e9);
}
