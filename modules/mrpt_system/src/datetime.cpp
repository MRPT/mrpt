/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2025, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

#include <mrpt/core/config.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/system/config.h>
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
#include <iostream>

using namespace mrpt;
using namespace mrpt::system;
using namespace std;

constexpr auto EPOCH_OFFSET = static_cast<uint64_t>(116444736) * 1000000000;

// Required to ensure INVALID_TIMESTAMP returns a "const T&":
const TTimeStamp& mrpt::system::InvalidTimeStamp()
{
  static thread_local TTimeStamp t = TTimeStamp();
  return t;
}

void mrpt::system::timestampToParts(TTimeStamp t, TTimeParts& p, bool localTime)
{
  const double T = mrpt::Clock::toDouble(t);
  double sec_frac = T - floor(T);
  ASSERT_(sec_frac < 1.0);

  const auto tt = time_t(T);

#if !defined(HAVE_LOCALTIME_R)
  struct tm* parts = localTime ? localtime(&tt) : gmtime(&tt);
#else
  tm myTm{};
  tm* parts = localTime ? localtime_r(&tt, &myTm) : gmtime_r(&tt, &myTm);
#endif

  ASSERTMSG_(parts, "Malformed timestamp");

  p.year = static_cast<uint16_t>(parts->tm_year + 1900);
  p.month = static_cast<uint8_t>(parts->tm_mon + 1);
  p.day = static_cast<uint8_t>(parts->tm_mday);
  p.day_of_week = static_cast<uint8_t>(parts->tm_wday + 1);
  p.daylight_saving = parts->tm_isdst;
  p.hour = static_cast<uint8_t>(parts->tm_hour);
  p.minute = static_cast<uint8_t>(parts->tm_min);
  p.second = parts->tm_sec + sec_frac;
}

namespace internal
{
/*---------------------------------------------------------------
          timegm
  ---------------------------------------------------------------*/
#ifdef HAVE_TIMEGM
time_t my_timegm(struct tm* tm) { return ::timegm(tm); }
#else
// Version for MSVC>=2005, which lacks "timegm"
#ifdef HAVE_MKGMTIME
time_t my_timegm(struct tm* tm) { return ::_mkgmtime(tm); }
#else
// generic version, slower but probably not used in any modern compiler!
time_t my_timegm(struct tm* tm)
{
  static std::mutex cs;
  std::lock_guard<std::mutex> lock(cs);

  time_t ret;
  char tz[256];

  /* save current timezone and set UTC */
  char* org_tz = getenv("TZ");
  if (org_tz) os::strcpy(tz, sizeof(tz), org_tz);

  putenv("TZ=UTC"); /* use Coordinated Universal Time (i.e. zero offset) */
  tzset();

  ret = mktime(tm);
  if (org_tz)
  {
    char buf[256];
    mrpt::system::os::sprintf(buf, sizeof(buf), "TZ=%s", tz);
    putenv(buf);
  }
  else
    putenv("TZ=");
  tzset();

  return ret;
}

#endif
#endif  // HAVE_TIMEGM
}  // namespace internal

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

  time_t tt = ::internal::my_timegm(&parts);  // Local time: mktime

  return mrpt::Clock::fromDouble(double(tt) + sec_frac);
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

  return mrpt::Clock::fromDouble(double(tt) + sec_frac);
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
  const auto milSecs = static_cast<unsigned int>(1000 * (timeSeconds - floor(timeSeconds)));

  if (nDays > 0)
  {
    s += mrpt::format("%udays ", nDays);
  }
  if (nHours > 0)
  {
    s += mrpt::format("%uh ", nHours);
  }
  if (nMins > 0)
  {
    s += mrpt::format("%02umin ", nMins);
  }
  s += mrpt::format("%02u.%03us", nSecs, milSecs);
  return s;
}

namespace
{
unsigned int calcSecFractions(const uint64_t tmp)
{
  return static_cast<unsigned int>(1e6 * static_cast<double>(tmp % 10000000) / 1e7);
}
}  // namespace

/*---------------------------------------------------------------
  Convert a timestamp into this textual form: YEAR/MONTH/DAY,HH:MM:SS.MMM
  ---------------------------------------------------------------*/
string mrpt::system::dateTimeToString(const mrpt::system::TTimeStamp t)
{
  if (t == INVALID_TIMESTAMP)
  {
    return string("INVALID_TIMESTAMP");
  }

  const uint64_t tmp = static_cast<uint64_t>(t.time_since_epoch().count()) - EPOCH_OFFSET;
  const time_t auxTime = static_cast<time_t>(tmp / 10000000U);
  const auto secFractions = calcSecFractions(tmp);
  tm* ptm = gmtime(&auxTime);

  if (!ptm)
  {
    return std::string("(Malformed timestamp)");
  }

  return format(
      "%u/%02u/%02u,%02u:%02u:%02u.%06u", 1900 + ptm->tm_year, ptm->tm_mon + 1, ptm->tm_mday,
      ptm->tm_hour, ptm->tm_min, static_cast<unsigned int>(ptm->tm_sec), secFractions);
}

/*---------------------------------------------------------------
  Convert a timestamp into this textual form (in local time):
    YEAR/MONTH/DAY,HH:MM:SS.MMM
  ---------------------------------------------------------------*/
string mrpt::system::dateTimeLocalToString(const mrpt::system::TTimeStamp t)
{
  if (t == INVALID_TIMESTAMP)
  {
    return string("INVALID_TIMESTAMP");
  }

  const uint64_t tmp = static_cast<uint64_t>(t.time_since_epoch().count()) - EPOCH_OFFSET;
  const time_t auxTime = static_cast<time_t>(tmp / 10000000U);
  const auto secFractions = calcSecFractions(tmp);

#if !defined(HAVE_LOCALTIME_R)
  tm* ptm = localtime(&auxTime);
#else
  tm myTm{};
  tm* ptm = localtime_r(&auxTime, &myTm);
#endif

  if (!ptm)
  {
    return "(Malformed timestamp)";
  }

  return format(
      "%u/%02u/%02u,%02u:%02u:%02u.%06u", 1900 + ptm->tm_year, ptm->tm_mon + 1, ptm->tm_mday,
      ptm->tm_hour, ptm->tm_min, static_cast<unsigned int>(ptm->tm_sec), secFractions);
}

/*---------------------------------------------------------------
            extractDayTimeFromTimestamp
  ---------------------------------------------------------------*/
double mrpt::system::extractDayTimeFromTimestamp(const mrpt::system::TTimeStamp tt)
{
  MRPT_START
  ASSERT_(tt != INVALID_TIMESTAMP);

  const uint64_t tmp = static_cast<uint64_t>(tt.time_since_epoch().count()) - EPOCH_OFFSET;
  const time_t auxTime = static_cast<time_t>(tmp / 10000000U);

  tm* ptm = gmtime(&auxTime);
  ASSERTMSG_(ptm, "Malformed timestamp");
  return ptm->tm_hour * 3600.0 + ptm->tm_min * 60.0 + ptm->tm_sec;
  MRPT_END
}

/*---------------------------------------------------------------
  Convert a timestamp into this textual form: HH:MM:SS.MMM
  ---------------------------------------------------------------*/
string mrpt::system::timeLocalToString(
    const mrpt::system::TTimeStamp tt, unsigned int secondFractionDigits)
{
  if (tt == INVALID_TIMESTAMP)
  {
    return string("INVALID_TIMESTAMP");
  }

  const uint64_t tmp = static_cast<uint64_t>(tt.time_since_epoch().count()) - EPOCH_OFFSET;
  const time_t auxTime = static_cast<time_t>(tmp / 10000000U);

#if !defined(HAVE_LOCALTIME_R)
  const tm* ptm = localtime(&auxTime);
#else
  tm myTm{};
  const tm* ptm = localtime_r(&auxTime, &myTm);
#endif

  auto secFractions = calcSecFractions(tmp);
  // We start with 10^{-6} second units: reduce if requested by user:
  const unsigned int user_secondFractionDigits = secondFractionDigits;
  while (secondFractionDigits++ < 6)
  {
    secFractions = secFractions / 10;
  }

  return format(
      "%02u:%02u:%02u.%0*u", ptm->tm_hour, ptm->tm_min, static_cast<unsigned int>(ptm->tm_sec),
      user_secondFractionDigits, secFractions);
}

/*---------------------------------------------------------------
  Convert a timestamp into this textual form: HH:MM:SS.MMM
  ---------------------------------------------------------------*/
string mrpt::system::timeToString(const mrpt::system::TTimeStamp tt)
{
  if (tt == INVALID_TIMESTAMP)
  {
    return string("INVALID_TIMESTAMP");
  }

  const uint64_t tmp = static_cast<uint64_t>(tt.time_since_epoch().count()) - EPOCH_OFFSET;
  const time_t auxTime = static_cast<time_t>(tmp / 10000000U);

  auto secFractions = calcSecFractions(tmp);
  tm* ptm = gmtime(&auxTime);
  if (!ptm)
  {
    return string("(Malformed timestamp)");
  }

  return format(
      "%02u:%02u:%02u.%06u", ptm->tm_hour, ptm->tm_min, static_cast<unsigned int>(ptm->tm_sec),
      secFractions);
}

/*---------------------------------------------------------------
  Convert a timestamp into this textual form: YEAR/MONTH/DAY
  ---------------------------------------------------------------*/
string mrpt::system::dateToString(const mrpt::system::TTimeStamp tt)
{
  if (tt == INVALID_TIMESTAMP)
  {
    return string("INVALID_TIMESTAMP");
  }

  const uint64_t tmp = static_cast<uint64_t>(tt.time_since_epoch().count()) - EPOCH_OFFSET;
  const time_t auxTime = static_cast<time_t>(tmp / 10000000U);

  tm* ptm = gmtime(&auxTime);
  if (!ptm)
  {
    return string("(Malformed timestamp)");
  }

  return format("%u/%02u/%02u", 1900 + ptm->tm_year, ptm->tm_mon + 1, ptm->tm_mday);
}

namespace
{
std::string implIntervalFormat(const double seconds)
{
  using namespace std::string_literals;

  if (seconds >= 365 * 24 * 3600)
  {
    const int i = static_cast<int>(seconds / (365 * 24 * 3600));
    return format("%i year%s", i, i > 1 ? "s" : "") + ", "s +
           implIntervalFormat(std::fmod(seconds, (365 * 24 * 3600)));
  }
  if (seconds >= 24 * 3600)
  {
    const int i = static_cast<int>(seconds / (24 * 3600));
    return format("%i day%s", i, i > 1 ? "s" : "") + ", "s +
           implIntervalFormat(std::fmod(seconds, (24 * 3600)));
  }

  if (seconds >= 3600)
  {
    const int i = static_cast<int>(seconds / 3600);
    return format("%i hour%s", i, i > 1 ? "s" : "") + ", "s +
           implIntervalFormat(std::fmod(seconds, 3600));
  }

  if (seconds >= 60)
  {
    const int i = static_cast<int>(seconds / 60);
    return format("%i minute%s", i, i > 1 ? "s" : "") + ", "s +
           implIntervalFormat(std::fmod(seconds, 60));
  }

  if (seconds >= 1)
  {
    return format("%.2f sec", seconds);
  }

  if (seconds >= 1e-3)
  {
    return format("%.2f ms", seconds * 1e3);
  }

  if (seconds >= 1e-6)
  {
    return format("%.2f us", seconds * 1e6);
  }

  return format("%.2f ns", seconds * 1e9);
}
}  // namespace

std::string mrpt::system::intervalFormat(const double seconds)
{
  return implIntervalFormat(seconds);
}

std::ostream& mrpt::system::operator<<(std::ostream& o, const TTimeStamp& t)
{
  const auto v = static_cast<uint64_t>(t.time_since_epoch().count());
  o << v;
  return o;
}
