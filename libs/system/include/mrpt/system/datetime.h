/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/core/Clock.h>
#include <mrpt/core/exceptions.h>

#include <cstdint>
#include <iosfwd>
#include <string>

namespace mrpt::system
{
/** @defgroup time_date Time and date functions.
 * Header: `#include <mrpt/system/datetime.h>`.
 * Library: \ref mrpt_system_grp
 *
 * Defines types and functions to handle cross-platform timestamps. The basic
 * type is mrpt::system::TTimeStamp, representing a high-resolution (100ns)
 * Clock::time_point, compatible with all C++11 std::chrono functions.
 *
 * There are also functions to convert forth and back to a `double`
 * representation of timestamps: numbers just like UNIX epoch timestamps but
 * with decimals for the fractionary part of seconds.
 *
 * \ingroup mrpt_system_grp
 * @{ */

/** A system independent time type, it holds the the number of 100-nanosecond
 * intervals since January 1, 1601 (UTC) as a mrpt::Clock::time_point
 * (uint64_t).
 * \sa system::getCurrentTime, system::timeDifference, INVALID_TIMESTAMP,
 * TTimeParts
 */
using TTimeStamp = mrpt::Clock::time_point;

/** Required to ensure INVALID_TIMESTAMP returns a "const T&"
 *  \note (New in MRPT 2.3.3)
 */
const TTimeStamp& InvalidTimeStamp();

/** Represents an invalid timestamp, where applicable.
 *
 * \note It returns a const reference to a thread_local static object of type
 * mrpt::Clock::time_point initialized with the default constructor, which is
 * used as the reference "invalid" timestamp.
 */
#define INVALID_TIMESTAMP mrpt::system::InvalidTimeStamp()

/** The parts of a date/time, like the standard `tm` but with fractional
 * (`double`) seconds. \sa TTimeStamp, timestampToParts, buildTimestampFromParts
 */
struct TTimeParts
{
	uint16_t year{0};  //!< The year (e.g. 2021)
	uint8_t month{0};  //!< Month (1-12)
	uint8_t day{0};	 //!<  Day (1-31)
	uint8_t hour{0};  //!<  Hour (0-23)
	uint8_t minute{0};	//!<  Minute (0-59)
	double second{0};  //!<  Seconds (0.0000-59.9999)
	uint8_t day_of_week{0};	 //!< Day of week (1:Sunday, 7:Saturday)
	int daylight_saving{0};
};

/** Builds a timestamp from the parts (Parts are in UTC)
 * \sa timestampToParts
 */
mrpt::system::TTimeStamp buildTimestampFromParts(
	const mrpt::system::TTimeParts& p);

/** Builds a timestamp from the parts (Parts are in local time)
 * \sa timestampToParts, buildTimestampFromParts
 */
mrpt::system::TTimeStamp buildTimestampFromPartsLocalTime(
	const mrpt::system::TTimeParts& p);

/** Gets the individual parts of a date/time (days, hours, minutes, seconds) -
 * UTC time or local time
 * \sa buildTimestampFromParts
 */
void timestampToParts(TTimeStamp t, TTimeParts& p, bool localTime = false);

/** Returns the time difference from t1 to t2 (positive if t2 is posterior to
 * t1), in seconds  */
inline double timeDifference(
	const mrpt::system::TTimeStamp& t_first,
	const mrpt::system::TTimeStamp& t_later)
{
	MRPT_START
	ASSERT_(t_later != INVALID_TIMESTAMP);
	ASSERT_(t_first != INVALID_TIMESTAMP);
	return 1e-6 *
		std::chrono::duration_cast<std::chrono::microseconds>(t_later - t_first)
			.count();
	MRPT_END
}

/** Shifts a timestamp the given amount of seconds (>0: forwards in time, <0:
 * backwards)  */
inline mrpt::system::TTimeStamp timestampAdd(
	const mrpt::system::TTimeStamp tim, const double num_seconds)
{
	return tim +
		std::chrono::microseconds(static_cast<int64_t>(num_seconds * 1e6));
}

/** Returns a formated string with the given time difference (passed as the
 * number of seconds), as a string [H]H:MM:SS.MILLISECONDS
 * \sa unitsFormat
 */
std::string formatTimeInterval(const double timeSeconds);

/** Convert a timestamp into this textual form (UTC time):
 * YEAR/MONTH/DAY,HH:MM:SS.MMM
 * \sa dateTimeLocalToString
 */
std::string dateTimeToString(const mrpt::system::TTimeStamp t);

/** Convert a timestamp into this textual form (in local time):
 * YEAR/MONTH/DAY,HH:MM:SS.MMM
 * \sa dateTimeToString
 */
std::string dateTimeLocalToString(const mrpt::system::TTimeStamp t);

/** Convert a timestamp into this textual form: YEAR/MONTH/DAY
 */
std::string dateToString(const mrpt::system::TTimeStamp t);

/** Returns the number of seconds ellapsed from midnight in the given timestamp
 */
double extractDayTimeFromTimestamp(const mrpt::system::TTimeStamp t);

/** Convert a timestamp into this textual form (UTC): HH:MM:SS.MMMMMM
 */
std::string timeToString(const mrpt::system::TTimeStamp t);

/** Convert a timestamp into this textual form (in local time): HH:MM:SS.MMMMMM
 */
std::string timeLocalToString(
	const mrpt::system::TTimeStamp t, unsigned int secondFractionDigits = 6);

/** This function implements time interval formatting: Given a time in seconds,
 * it will return a string describing the interval with the most appropriate
 * unit.
 * E.g.:
 *  - "1 year, 3 days, 4 minutes"
 *  - "3 days, 8 hours"
 *  - "9 hours, 4 minutes, 4.3 sec",
 *  - "3.34 sec"
 *  - "178.1 ms"
 *  - "87.1 us"
 *
 * \sa unitsFormat
 */
std::string intervalFormat(const double seconds);

/** Textual representation of a TTimeStamp as the plain number in
 * time_since_epoch().count() */
std::ostream& operator<<(std::ostream& o, const TTimeStamp& t);

/** @} */

}  // namespace mrpt::system
