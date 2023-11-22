/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/system/CTimeLogger.h>

#include <chrono>
#include <mutex>
#include <thread>

namespace
{
void doTimLogEntry(
	mrpt::system::CTimeLogger& tl, const char* name, const int ms)
{
	tl.enter(name);
	std::this_thread::sleep_for(std::chrono::milliseconds(ms));
	tl.leave(name);
}
}  // namespace

TEST(CTimeLogger, getLastTime)
{
	mrpt::system::CTimeLogger tl;
	tl.enable(true);
	doTimLogEntry(tl, "foo", 10);

	EXPECT_GT(tl.getLastTime("foo"), 5e-3);
	EXPECT_EQ(tl.getLastTime("bar"), 0);

	tl.clear();
	doTimLogEntry(tl, "foo2", 10);
	EXPECT_GT(tl.getLastTime("foo2"), 5e-3);
	EXPECT_EQ(tl.getLastTime("foo"), 0);

	tl.clear(true);
	doTimLogEntry(tl, "foo3", 10);
	EXPECT_GT(tl.getLastTime("foo3"), 5e-3);
	EXPECT_EQ(tl.getLastTime("foo2"), 0);

	tl.clear();
	tl.enable(false);
	doTimLogEntry(tl, "foo", 10);
	EXPECT_EQ(tl.getLastTime("foo"), 0);

	tl.clear(true);	 // to silent console output upon dtor
}

TEST(CTimeLogger, getMeanTime)
{
	mrpt::system::CTimeLogger tl;
	doTimLogEntry(tl, "foo", 10);
	doTimLogEntry(tl, "foo", 300);

	EXPECT_GT(tl.getMeanTime("foo"), 100e-3);

	tl.clear(true);	 // to silent console output upon dtor
}

TEST(CTimeLogger, printStats)
{
	mrpt::system::CTimeLogger tl;
	doTimLogEntry(tl, "foo", 1);
	doTimLogEntry(tl, "foo.1", 1);
	doTimLogEntry(tl, "foo.2", 1);
	doTimLogEntry(tl, "foo.3", 1);
	doTimLogEntry(tl, "bar", 1);
	doTimLogEntry(tl, "bar.1", 1);
	doTimLogEntry(tl, "bar.2", 1);
	doTimLogEntry(tl, "bar.3", 1);

	const std::string s = tl.getStatsAsText();
	tl.clear(true);
	EXPECT_EQ(std::count(s.begin(), s.end(), '\n'), 12U);
}

// Used to fix a bug in results table generation (Aug/2020)
TEST(CTimeLogger, printStatsFaulty)
{
	mrpt::system::CTimeLogger tl;
	doTimLogEntry(tl, "foo", 1);
	doTimLogEntry(tl, "foo.1", 1);
	doTimLogEntry(tl, "foo.2", 1);
	doTimLogEntry(tl, "foo.3", 1);
	doTimLogEntry(tl, "bar.1", 1);
	doTimLogEntry(tl, "bar.2", 1);
	doTimLogEntry(tl, "bar.3", 1);
	doTimLogEntry(tl, "zoo.1", 1);
	doTimLogEntry(tl, "zoo.2", 1);
	doTimLogEntry(tl, "zoo.3", 1);

	const std::string s = tl.getStatsAsText();
	tl.clear(true);
	EXPECT_EQ(std::count(s.begin(), s.end(), '\n'), 14U);
}

TEST(CTimeLogger, multithread)
{
	mrpt::system::CTimeLogger tl;
	std::vector<std::thread> ths;
	std::mutex mtx;

	mtx.lock();

	for (int i = 0; i < 20; i++)
	{
		ths.push_back(std::thread([i, &mtx, &tl]() {
			mtx.lock();
			mtx.unlock();
			doTimLogEntry(tl, mrpt::format("foo%i", i % 5).c_str(), 10);
		}));
	}

	mtx.unlock();  // now, all threads will run

	for (auto& t : ths)
		if (t.joinable()) t.join();

	const std::string s = tl.getStatsAsText();
	tl.clear(true);	 // to silent console output upon dtor
	EXPECT_EQ(std::count(s.begin(), s.end(), '\n'), 9U);
}
