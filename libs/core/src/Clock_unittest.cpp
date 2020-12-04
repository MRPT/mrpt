/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/core/Clock.h>

#include <chrono>
#include <thread>

static void test_delay()
{
	const double t0 = mrpt::Clock::nowDouble();
	std::this_thread::sleep_for(std::chrono::milliseconds(10));
	const double t1 = mrpt::Clock::nowDouble();

	EXPECT_GT(t1 - t0, 0.008);	// ideally, near 0.010
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
	const double t0 = mrpt::Clock::nowDouble();
	mrpt::Clock::setActiveClock(mrpt::Clock::Source::Monotonic);

	std::this_thread::sleep_for(std::chrono::milliseconds(10));
	const double t1 = mrpt::Clock::nowDouble();
	mrpt::Clock::setActiveClock(mrpt::Clock::Source::Realtime);

	std::this_thread::sleep_for(std::chrono::milliseconds(10));
	const double t2 = mrpt::Clock::nowDouble();

	EXPECT_GT(t1 - t0, 0.008);	// ideally, near 0.010
	EXPECT_LT(t1 - t0, 5.0);  // just detect it's not a crazy number

	EXPECT_GT(t2 - t1, 0.008);	// ideally, near 0.010
	EXPECT_LT(t2 - t1, 5.0);  // just detect it's not a crazy number
}

TEST(clock, checkSynchEpoch)
{
	for (int i = 0; i < 20; i++)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(5));
		const int64_t err = mrpt::Clock::resetMonotonicToRealTimeEpoch();

		// it should be a really small number in a regular computer,
		// but we set the threshold much higher due to spurious errors
		// when running unit tests in VMs (build farms)
		EXPECT_LT(std::abs(err), 70000);
	}
}

TEST(clock, simulatedTime)
{
	const auto t0 = mrpt::Clock::now();
	const auto prevSrc = mrpt::Clock::getActiveClock();
	// Enable simulated time:
	mrpt::Clock::setActiveClock(mrpt::Clock::Simulated);

	// Check that time is stopped:
	mrpt::Clock::setSimulatedTime(t0);
	const auto t1 = mrpt::Clock::now();
	EXPECT_EQ(t0, t1);
	std::this_thread::sleep_for(std::chrono::milliseconds(10));
	const auto t2 = mrpt::Clock::now();
	EXPECT_EQ(t0, t2);

	// Check time moving forward:
	auto tset2 = t0 + std::chrono::milliseconds(1000);
	mrpt::Clock::setSimulatedTime(tset2);
	const auto t3 = mrpt::Clock::now();
	EXPECT_EQ(tset2, t3);

	// Restore
	mrpt::Clock::setActiveClock(prevSrc);
}
