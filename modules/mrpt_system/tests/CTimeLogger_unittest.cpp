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
#include <mrpt/core/format.h>
#include <mrpt/system/CTimeLogger.h>
#include <mrpt/system/filesystem.h>

#include <atomic>
#include <chrono>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace
{
void doTimLogEntry(mrpt::system::CTimeLogger& tl, const char* name, const int ms)
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

  tl.clear(true);  // to silent console output upon dtor
}

TEST(CTimeLogger, getMeanTime)
{
  mrpt::system::CTimeLogger tl;
  doTimLogEntry(tl, "foo", 10);
  doTimLogEntry(tl, "foo", 300);

  EXPECT_GT(tl.getMeanTime("foo"), 100e-3);

  tl.clear(true);  // to silent console output upon dtor
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
    ths.push_back(std::thread(
        [i, &mtx, &tl]()
        {
          mtx.lock();
          mtx.unlock();
          doTimLogEntry(tl, mrpt::format("foo%i", i % 5).c_str(), 10);
        }));
  }

  mtx.unlock();  // now, all threads will run

  for (auto& t : ths)
  {
    if (t.joinable())
    {
      t.join();
    }
  }

  const std::string s = tl.getStatsAsText();
  tl.clear(true);  // to silent console output upon dtor
  EXPECT_EQ(std::count(s.begin(), s.end(), '\n'), 9U);
}

// Reporting/saving must be safe to call from one thread while other threads
// keep logging (including registering brand-new sections). Before the fix, the
// report methods iterated the live container without synchronization against
// the writers, which is a data race (crash under TSan/ASan).
TEST(CTimeLogger, concurrentDumpWhileLogging)
{
  mrpt::system::CTimeLogger tl;
  tl.enableKeepWholeHistory(true);

  std::atomic_bool stop{false};
  std::vector<std::thread> ths;

  // Writers: each thread keeps allocating/updating a growing set of sections.
  for (int t = 0; t < 8; t++)
  {
    ths.emplace_back(
        [t, &tl, &stop]()
        {
          int i = 0;
          while (!stop)
          {
            const std::string name = mrpt::format("thread%d_sec%d", t, (i++) % 40);
            tl.enter(name);
            tl.leave(name);
          }
        });
  }

  // Readers: hammer all the report/save paths concurrently with the writers.
  const std::string csv = mrpt::system::getTempFileName();
  for (int r = 0; r < 3; r++)
  {
    ths.emplace_back(
        [&tl, &csv, &stop]()
        {
          while (!stop)
          {
            (void)tl.getStatsAsText();
            std::map<std::string, mrpt::system::CTimeLogger::TCallStats> stats;
            tl.getStats(stats);
            tl.saveToCSVFile(csv);
          }
        });
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  stop = true;
  for (auto& th : ths)
  {
    th.join();
  }

  // If we got here without crashing/deadlocking, the test passed. Sanity check:
  EXPECT_FALSE(tl.getStatsAsText().empty());
  tl.clear(true);  // to silent console output upon dtor
}

// clear(false) must also be safe against concurrent logging.
TEST(CTimeLogger, concurrentClearWhileLogging)
{
  mrpt::system::CTimeLogger tl;
  std::atomic_bool stop{false};
  std::vector<std::thread> ths;

  for (int t = 0; t < 6; t++)
  {
    ths.emplace_back(
        [t, &tl, &stop]()
        {
          int i = 0;
          while (!stop)
          {
            const std::string name = mrpt::format("t%d_s%d", t, (i++) % 20);
            tl.enter(name);
            tl.leave(name);
          }
        });
  }

  for (int k = 0; k < 200; k++)
  {
    tl.clear(false);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  stop = true;
  for (auto& th : ths)
  {
    th.join();
  }
  tl.clear(true);  // to silent console output upon dtor
}
