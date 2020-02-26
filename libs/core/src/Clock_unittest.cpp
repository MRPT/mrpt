/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/core/Clock.h>
#include <chrono>
#include <thread>

static void test_delay()
{
	const double t0 = mrpt::Clock::toDouble(mrpt::Clock::now());
	std::this_thread::sleep_for(std::chrono::milliseconds(10));
	const double t1 = mrpt::Clock::toDouble(mrpt::Clock::now());

	EXPECT_GT(t1 - t0, 0.008);  // ideally, near 0.010
	EXPECT_LT(t1 - t0, 5.0);  // just detect it's not a crazy number
}

TEST(clock, delay_Realtime)
{
	// Default:
	test_delay();

	// Monotonic:
	mrpt::Clock::setActiveClock(mrpt::Clock::Source::Monotonic);
	test_delay();

	// Realtime:
	mrpt::Clock::setActiveClock(mrpt::Clock::Source::Realtime);
	test_delay();
}

TEST(clock, changeSource)
{
	const double t0 = mrpt::Clock::toDouble(mrpt::Clock::now());
	mrpt::Clock::setActiveClock(mrpt::Clock::Source::Monotonic);

	std::this_thread::sleep_for(std::chrono::milliseconds(10));
	const double t1 = mrpt::Clock::toDouble(mrpt::Clock::now());
	mrpt::Clock::setActiveClock(mrpt::Clock::Source::Realtime);

	std::this_thread::sleep_for(std::chrono::milliseconds(10));
	const double t2 = mrpt::Clock::toDouble(mrpt::Clock::now());

	EXPECT_GT(t1 - t0, 0.008);  // ideally, near 0.010
	EXPECT_LT(t1 - t0, 5.0);  // just detect it's not a crazy number

	EXPECT_GT(t2 - t1, 0.008);  // ideally, near 0.010
	EXPECT_LT(t2 - t1, 5.0);  // just detect it's not a crazy number
}

TEST(clock, checkSynchEpoch)
{
	for (int i = 0; i < 20; i++)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(5));
		const int64_t err = mrpt::Clock::resetMonotonicToRealTimeEpoch();
		EXPECT_LT(std::abs(err), 5000);
	}
}
