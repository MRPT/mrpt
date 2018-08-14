/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include <mrpt/system/datetime.h>
#include <mrpt/core/Clock.h>
#include <gtest/gtest.h>

#include <thread>

TEST(DateTime, dateTimeVsClock)
{
	using namespace std::chrono_literals;
	auto now1 = mrpt::Clock::now().time_since_epoch();
	std::this_thread::sleep_for(1ms);
	auto now_timestamp1 = mrpt::system::getCurrentTime();
	EXPECT_LT(now1.count(), now_timestamp1.time_since_epoch().count());

	auto now_timestamp2 = mrpt::system::getCurrentTime();
	std::this_thread::sleep_for(1ms);
	auto now2 = mrpt::Clock::now().time_since_epoch();
	EXPECT_GT(now2.count(), now_timestamp2.time_since_epoch().count());
}

TEST(DateTime, time_t_forth_back)
{
	const double td5 = 1534142320.5;
	const time_t t = 1534142320;

	auto t1 = mrpt::system::time_tToTimestamp(td5);
	auto t2 = mrpt::system::time_tToTimestamp(t);
	EXPECT_NEAR(
		std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t2).count(),
		500, 1);

	const double t1d = mrpt::system::timestampTotime_t(t1);
	const double t2d = mrpt::system::timestampTotime_t(t2);
	EXPECT_NEAR((t1d - t2d), 0.5, 1e-3);
}

TEST(DateTime, fixed_date_check)
{
	const uint64_t t_raw = 127822463930948526;
	const auto t = mrpt::Clock::time_point(mrpt::Clock::duration(t_raw));
	const std::string s = mrpt::system::dateTimeToString(t);
	EXPECT_EQ(std::string("2006/01/20,15:59:53.094852"), s);

	const double t_d = mrpt::system::timestampTotime_t(t);
	EXPECT_NEAR(1137772793.09485, t_d, 1e-5);
}

TEST(DateTime, double_to_from)
{
	auto t1 = mrpt::system::now();
	const double d1 = mrpt::system::timestampToDouble(t1);
	auto t2 = mrpt::Clock::fromDouble(d1);
	EXPECT_NEAR(
		mrpt::system::timestampToDouble(t1),
		mrpt::system::timestampToDouble(t2), 1e-4);
}
