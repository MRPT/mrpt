/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include <mrpt/system/datetime.h>
#include <mrpt/system/Clock.h>
#include <gtest/gtest.h>

#include <thread>

TEST(DateTime, dateTimeVsClock)
{
	using namespace std::chrono_literals;
	auto now1 = mrpt::system::Clock::now().time_since_epoch();
	std::this_thread::sleep_for(1ms);
	auto now_timestamp1 = mrpt::system::getCurrentTime();
	EXPECT_LT(now1.count(), now_timestamp1);

	auto now_timestamp2 = mrpt::system::getCurrentTime();
	std::this_thread::sleep_for(1ms);
	auto now2 = mrpt::system::Clock::now().time_since_epoch();
	EXPECT_GT(now2.count(), now_timestamp2);
}
