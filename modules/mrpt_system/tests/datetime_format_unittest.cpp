/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

#include <gtest/gtest.h>
#include <mrpt/system/datetime.h>

#include <sstream>
#include <string>

using namespace mrpt::system;

namespace
{
/** 2020-06-15 12:34:56.789 UTC, as a fixed reference for the formatters. */
TTimeStamp referenceTimestamp()
{
  TTimeParts p;
  p.year = 2020;
  p.month = 6;
  p.day = 15;
  p.hour = 12;
  p.minute = 34;
  p.second = 56.789;
  p.day_of_week = 0;
  p.daylight_saving = 0;
  return buildTimestampFromParts(p);
}
}  // namespace

TEST(DateTime, partsRoundTrip)
{
  const auto t = referenceTimestamp();

  TTimeParts p;
  timestampToParts(t, p);

  EXPECT_EQ(p.year, 2020);
  EXPECT_EQ(p.month, 6);
  EXPECT_EQ(p.day, 15);
  EXPECT_EQ(p.hour, 12);
  EXPECT_EQ(p.minute, 34);
  EXPECT_NEAR(p.second, 56.789, 1e-3);
}

TEST(DateTime, localTimePartsRoundTrip)
{
  const auto t = mrpt::Clock::now();

  TTimeParts p;
  timestampToParts(t, p, true /*localTime*/);
  const auto back = buildTimestampFromPartsLocalTime(p);

  // Sub-second precision is preserved through the round trip:
  EXPECT_NEAR(timeDifference(t, back), 0.0, 1e-3);
}

TEST(DateTime, dateAndTimeToString)
{
  const auto t = referenceTimestamp();

  const std::string dt = dateTimeToString(t);
  EXPECT_NE(dt.find("2020/06/15"), std::string::npos);
  EXPECT_NE(dt.find("12:34:56"), std::string::npos);

  EXPECT_NE(dateToString(t).find("2020/06/15"), std::string::npos);
  EXPECT_NE(timeToString(t).find("12:34:56"), std::string::npos);

  // The "local time" variants use whatever zone the machine is in, so only
  // check that they produce something of the right shape:
  EXPECT_FALSE(dateTimeLocalToString(t).empty());
  EXPECT_EQ(timeLocalToString(t).size(), timeToString(t).size());
}

TEST(DateTime, invalidTimestampsAreLabelled)
{
  const auto t = INVALID_TIMESTAMP;

  EXPECT_EQ(dateTimeToString(t), "INVALID_TIMESTAMP");
  EXPECT_EQ(dateTimeLocalToString(t), "INVALID_TIMESTAMP");
  EXPECT_EQ(timeToString(t), "INVALID_TIMESTAMP");
  EXPECT_EQ(timeLocalToString(t), "INVALID_TIMESTAMP");
  EXPECT_EQ(dateToString(t), "INVALID_TIMESTAMP");

  // ...but this one asserts instead of returning a sentinel:
  EXPECT_ANY_THROW((void)extractDayTimeFromTimestamp(t));

  EXPECT_EQ(InvalidTimeStamp(), INVALID_TIMESTAMP);
}

TEST(DateTime, extractDayTimeFromTimestamp)
{
  const auto t = referenceTimestamp();
  const double dayTime = extractDayTimeFromTimestamp(t);

  // Seconds elapsed since midnight, in whatever the local zone is: the value
  // must at least stay inside one day.
  EXPECT_GE(dayTime, 0.0);
  EXPECT_LT(dayTime, 24 * 3600.0);
}

TEST(DateTime, formatTimeInterval)
{
  // Only the components that are non-zero show up:
  EXPECT_EQ(formatTimeInterval(0.0), "00.000s");
  EXPECT_NE(formatTimeInterval(61.5).find("01min"), std::string::npos);
  EXPECT_NE(formatTimeInterval(3661.0).find("1h"), std::string::npos);

  const std::string longer = formatTimeInterval(2 * 24 * 3600 + 3661.0);
  EXPECT_NE(longer.find("2days"), std::string::npos);
  EXPECT_NE(longer.find("1h"), std::string::npos);

  // A negative interval is formatted by magnitude:
  EXPECT_EQ(formatTimeInterval(-61.5), formatTimeInterval(61.5));
}

TEST(DateTime, intervalFormatPicksAUnit)
{
  EXPECT_NE(intervalFormat(1e-9).find("ns"), std::string::npos);
  EXPECT_NE(intervalFormat(1e-4).find("us"), std::string::npos);
  EXPECT_NE(intervalFormat(1e-2).find("ms"), std::string::npos);
  EXPECT_NE(intervalFormat(2.5).find("sec"), std::string::npos);
  EXPECT_NE(intervalFormat(90.0).find("minute"), std::string::npos);
  EXPECT_NE(intervalFormat(3700.0).find("hour"), std::string::npos);

  // Plurals, and the recursive composition of larger units:
  const std::string days = intervalFormat(2 * 24 * 3600 + 3700.0);
  EXPECT_NE(days.find("2 days"), std::string::npos);
  EXPECT_NE(days.find("hour"), std::string::npos);

  const std::string years = intervalFormat(3.0 * 365 * 24 * 3600);
  EXPECT_NE(years.find("3 years"), std::string::npos);

  const std::string oneYear = intervalFormat(1.5 * 365 * 24 * 3600);
  EXPECT_NE(oneYear.find("1 year,"), std::string::npos);
}

TEST(DateTime, streamOperator)
{
  const auto t = referenceTimestamp();

  std::ostringstream ss;
  ss << t;
  EXPECT_FALSE(ss.str().empty());
  EXPECT_NE(ss.str(), "0");
}
