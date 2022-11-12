/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "system-precomp.h"	 // Precompiled headers
//
#include <mrpt/config.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/system/datetime.h>
#include <mrpt/system/os.h>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
//
#include <conio.h>
#include <direct.h>
#include <io.h>
#include <process.h>
#include <sys/utime.h>
#include <tlhelp32.h>
#else
#include <pthread.h>
#include <sys/select.h>
#include <sys/time.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
#include <utime.h>

#include <cerrno>
#endif

#include <sys/stat.h>
#include <sys/types.h>

#include <cmath>  // floor()
#include <ctime>
#include <iostream>	 // for the << operator

using namespace mrpt;
using namespace mrpt::system;
using namespace std;

// Required to ensure INVALID_TIMESTAMP returns a "const T&":
const TTimeStamp& mrpt::system::InvalidTimeStamp()
{
	static thread_local TTimeStamp t = TTimeStamp();
	return t;
}

mrpt::system::TTimeStamp mrpt::system::time_tToTimestamp(const time_t& t)
{
	return time_tToTimestamp(static_cast<double>(t));
}

void mrpt::system::timestampToParts(TTimeStamp t, TTimeParts& p, bool localTime)
{
	const double T = mrpt::system::timestampTotime_t(t);
	double sec_frac = T - floor(T);
	ASSERT_(sec_frac < 1.0);

	const auto tt = time_t(T);

#if !defined(HAVE_LOCALTIME_R)
	struct tm* parts = localTime ? localtime(&tt) : gmtime(&tt);
#else
	tm myTm;
	tm* parts = localTime ? localtime_r(&tt, &myTm) : gmtime_r(&tt, &myTm);
#endif

	ASSERTMSG_(parts, "Malformed timestamp");

	p.year = parts->tm_year + 1900;
	p.month = parts->tm_mon + 1;
	p.day = parts->tm_mday;
	p.day_of_week = parts->tm_wday + 1;
	p.daylight_saving = parts->tm_isdst;
	p.hour = parts->tm_hour;
	p.minute = parts->tm_min;
	p.second = parts->tm_sec + sec_frac;
}

/*---------------------------------------------------------------
					buildTimestampFromParts
  ---------------------------------------------------------------*/
TTimeStamp mrpt::system::buildTimestampFromParts(const TTimeParts& p)
{
	struct tm parts
	{
	};

	parts.tm_year = p.year - 1900;
	parts.tm_mon = p.month - 1;
	parts.tm_mday = p.day;
	parts.tm_wday = p.day_of_week - 1;
	parts.tm_isdst = p.daylight_saving;
	parts.tm_hour = p.hour;
	parts.tm_min = p.minute;
	parts.tm_sec = int(p.second);

	double sec_frac = p.second - parts.tm_sec;

	time_t tt = mrpt::system::os::timegm(&parts);  // Local time: mktime

	return mrpt::system::time_tToTimestamp(double(tt) + sec_frac);
}

/*---------------------------------------------------------------
					buildTimestampFromPartsLocalTime
  ---------------------------------------------------------------*/
TTimeStamp mrpt::system::buildTimestampFromPartsLocalTime(const TTimeParts& p)
{
	struct tm parts
	{
	};

	parts.tm_year = p.year - 1900;
	parts.tm_mon = p.month - 1;
	parts.tm_mday = p.day;
	parts.tm_wday = p.day_of_week - 1;
	parts.tm_isdst = p.daylight_saving;
	parts.tm_hour = p.hour;
	parts.tm_min = p.minute;
	parts.tm_sec = int(p.second);

	double sec_frac = p.second - parts.tm_sec;

	time_t tt = mktime(&parts);

	return mrpt::system::time_tToTimestamp(double(tt) + sec_frac);
}

/*---------------------------------------------------------------
					formatTimeInterval
  ---------------------------------------------------------------*/
string mrpt::system::formatTimeInterval(const double t)
{
	double timeSeconds = (t < 0) ? (-t) : t;
	std::string s;

	const auto nDays = static_cast<unsigned int>(timeSeconds / (3600 * 24));
	timeSeconds -= nDays * 3600 * 24;
	const auto nHours = static_cast<unsigned int>(timeSeconds / 3600);
	timeSeconds -= nHours * 3600;
	const auto nMins = static_cast<unsigned int>(timeSeconds / 60);
	const auto nSecs = static_cast<unsigned int>(timeSeconds) % 60;
	const auto milSecs =
		static_cast<unsigned int>(1000 * (timeSeconds - floor(timeSeconds)));

	if (nDays > 0) s += mrpt::format("%udays ", nDays);
	if (nHours > 0) s += mrpt::format("%uh ", nHours);
	if (nMins > 0) s += mrpt::format("%02umin ", nMins);
	s += mrpt::format("%02u.%03us", nSecs, milSecs);
	return s;
}

static unsigned int calcSecFractions(const uint64_t tmp)
{
	return static_cast<unsigned int>(
		1e6 * static_cast<double>(tmp % 10000000) / 1e7);
}

/*---------------------------------------------------------------
  Convert a timestamp into this textual form: YEAR/MONTH/DAY,HH:MM:SS.MMM
  ---------------------------------------------------------------*/
string mrpt::system::dateTimeToString(const mrpt::system::TTimeStamp t)
{
	if (t == INVALID_TIMESTAMP) return string("INVALID_TIMESTAMP");

	uint64_t tmp =
		(t.time_since_epoch().count() - ((uint64_t)116444736 * 1000000000));
	time_t auxTime = tmp / (uint64_t)10000000;
	auto secFractions = calcSecFractions(tmp);
	tm* ptm = gmtime(&auxTime);

	if (!ptm) return std::string("(Malformed timestamp)");

	return format(
		"%u/%02u/%02u,%02u:%02u:%02u.%06u", 1900 + ptm->tm_year,
		ptm->tm_mon + 1, ptm->tm_mday, ptm->tm_hour, ptm->tm_min,
		(unsigned int)ptm->tm_sec, secFractions);
}

/*---------------------------------------------------------------
  Convert a timestamp into this textual form (in local time):
	  YEAR/MONTH/DAY,HH:MM:SS.MMM
  ---------------------------------------------------------------*/
string mrpt::system::dateTimeLocalToString(const mrpt::system::TTimeStamp t)
{
	if (t == INVALID_TIMESTAMP) return string("INVALID_TIMESTAMP");

	uint64_t tmp =
		(t.time_since_epoch().count() - ((uint64_t)116444736 * 1000000000));
	time_t auxTime = tmp / (uint64_t)10000000;
	auto secFractions = calcSecFractions(tmp);

#if !defined(HAVE_LOCALTIME_R)
	tm* ptm = localtime(&auxTime);
#else
	tm myTm;
	tm* ptm = localtime_r(&auxTime, &myTm);
#endif

	if (!ptm) return "(Malformed timestamp)";

	return format(
		"%u/%02u/%02u,%02u:%02u:%02u.%06u", 1900 + ptm->tm_year,
		ptm->tm_mon + 1, ptm->tm_mday, ptm->tm_hour, ptm->tm_min,
		(unsigned int)ptm->tm_sec, secFractions);
}

/*---------------------------------------------------------------
						extractDayTimeFromTimestamp
  ---------------------------------------------------------------*/
double mrpt::system::extractDayTimeFromTimestamp(
	const mrpt::system::TTimeStamp tt)
{
	MRPT_START
	ASSERT_(tt != INVALID_TIMESTAMP);

	auto t = tt.time_since_epoch().count();
#ifdef _WIN32
	SYSTEMTIME sysT;
	FileTimeToSystemTime((FILETIME*)&t, &sysT);
	return sysT.wHour * 3600.0 + sysT.wMinute * 60.0 + sysT.wSecond +
		sysT.wMilliseconds * 0.001;
#else
	time_t auxTime =
		(t - ((uint64_t)116444736 * 1000000000)) / (uint64_t)10000000;
	tm* ptm = gmtime(&auxTime);
	ASSERTMSG_(ptm, "Malformed timestamp");
	return ptm->tm_hour * 3600.0 + ptm->tm_min * 60.0 + ptm->tm_sec;
#endif
	MRPT_END
}

/*---------------------------------------------------------------
  Convert a timestamp into this textual form: HH:MM:SS.MMM
  ---------------------------------------------------------------*/
string mrpt::system::timeLocalToString(
	const mrpt::system::TTimeStamp tt, unsigned int secondFractionDigits)
{
	if (tt == INVALID_TIMESTAMP) return string("INVALID_TIMESTAMP");
	auto t = tt.time_since_epoch().count();

	uint64_t tmp = (t - ((uint64_t)116444736 * 1000000000));
	const time_t auxTime = tmp / (uint64_t)10000000;

#if !defined(HAVE_LOCALTIME_R)
	const tm* ptm = localtime(&auxTime);
#else
	tm myTm;
	const tm* ptm = localtime_r(&auxTime, &myTm);
#endif

	auto secFractions = calcSecFractions(tmp);
	// We start with 10^{-6} second units: reduce if requested by user:
	const unsigned int user_secondFractionDigits = secondFractionDigits;
	while (secondFractionDigits++ < 6)
		secFractions = secFractions / 10;

	return format(
		"%02u:%02u:%02u.%0*u", ptm->tm_hour, ptm->tm_min,
		(unsigned int)ptm->tm_sec, user_secondFractionDigits, secFractions);
}

/*---------------------------------------------------------------
  Convert a timestamp into this textual form: HH:MM:SS.MMM
  ---------------------------------------------------------------*/
string mrpt::system::timeToString(const mrpt::system::TTimeStamp tt)
{
	if (tt == INVALID_TIMESTAMP) return string("INVALID_TIMESTAMP");
	auto t = tt.time_since_epoch().count();

	uint64_t tmp = (t - ((uint64_t)116444736 * 1000000000));
	time_t auxTime = tmp / (uint64_t)10000000;
	auto secFractions = calcSecFractions(tmp);
	tm* ptm = gmtime(&auxTime);
	if (!ptm) return string("(Malformed timestamp)");

	return format(
		"%02u:%02u:%02u.%06u", ptm->tm_hour, ptm->tm_min,
		(unsigned int)ptm->tm_sec, secFractions);
}

/*---------------------------------------------------------------
  Convert a timestamp into this textual form: YEAR/MONTH/DAY
  ---------------------------------------------------------------*/
string mrpt::system::dateToString(const mrpt::system::TTimeStamp tt)
{
	if (tt == INVALID_TIMESTAMP) return string("INVALID_TIMESTAMP");
	auto t = tt.time_since_epoch().count();

	uint64_t tmp = (t - ((uint64_t)116444736 * 1000000000));
	time_t auxTime = tmp / (uint64_t)10000000;
	tm* ptm = gmtime(&auxTime);
	if (!ptm) return string("(Malformed timestamp)");

	return format(
		"%u/%02u/%02u", 1900 + ptm->tm_year, ptm->tm_mon + 1, ptm->tm_mday);
}

static std::string implIntervalFormat(const double seconds)
{
	using namespace std::string_literals;

	if (seconds >= 365 * 24 * 3600)
	{
		const int i = static_cast<int>(seconds / (365 * 24 * 3600));
		return format("%i year%s", i, i > 1 ? "s" : "") + ", "s +
			implIntervalFormat(std::fmod(seconds, (365 * 24 * 3600)));
	}
	else if (seconds >= 24 * 3600)
	{
		const int i = static_cast<int>(seconds / (24 * 3600));
		return format("%i day%s", i, i > 1 ? "s" : "") + ", "s +
			implIntervalFormat(std::fmod(seconds, (24 * 3600)));
	}
	else if (seconds >= 3600)
	{
		const int i = static_cast<int>(seconds / 3600);
		return format("%i hour%s", i, i > 1 ? "s" : "") + ", "s +
			implIntervalFormat(std::fmod(seconds, 3600));
	}
	else if (seconds >= 60)
	{
		const int i = static_cast<int>(seconds / 60);
		return format("%i minute%s", i, i > 1 ? "s" : "") + ", "s +
			implIntervalFormat(std::fmod(seconds, 60));
	}
	else if (seconds >= 1)
		return format("%.2f sec", seconds);
	else if (seconds >= 1e-3)
		return format("%.2f ms", seconds * 1e3);
	else if (seconds >= 1e-6)
		return format("%.2f us", seconds * 1e6);
	else
		return format("%.2f ns", seconds * 1e9);
}

std::string mrpt::system::intervalFormat(const double seconds)
{
	return implIntervalFormat(seconds);
}

std::ostream& mrpt::system::operator<<(std::ostream& o, const TTimeStamp& t)
{
	const uint64_t v = t.time_since_epoch().count();
	o << v;
	return o;
}
