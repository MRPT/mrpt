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
#include <mrpt/system/COutputLogger.h>

#include <atomic>
#include <chrono>
#include <thread>
#include <vector>

TEST(COutputLogger, RegisterDeregisterCallback)
{
  mrpt::system::COutputLogger log;
  log.setVerbosityLevel(mrpt::system::LVL_DEBUG);

  int callCount = 0;
  auto cb = [&](std::string_view, const mrpt::system::VerbosityLevel, std::string_view,
                const mrpt::Clock::time_point&) { callCount++; };

  log.logRegisterCallback(cb);
  log.logStr(mrpt::system::LVL_INFO, "hello");
  EXPECT_EQ(callCount, 1);

  EXPECT_TRUE(log.logDeregisterCallback(cb));
  log.logStr(mrpt::system::LVL_INFO, "world");
  EXPECT_EQ(callCount, 1);  // no further calls after deregistering

  EXPECT_FALSE(log.logDeregisterCallback(cb));  // already removed
}

// Regression test for a data race between logStr() iterating the callback
// list and logRegisterCallback()/logDeregisterCallback() mutating it from
// another thread. Run under TSan to catch any regression; otherwise this
// simply exercises the code paths without crashing.
TEST(COutputLogger, ConcurrentRegisterAndLog)
{
  mrpt::system::COutputLogger log;
  log.setVerbosityLevel(mrpt::system::LVL_DEBUG);
  log.logging_enable_console_output = false;

  std::atomic<bool> stop{false};
  std::atomic<int> callCount{0};

  auto cb = [&](std::string_view, const mrpt::system::VerbosityLevel, std::string_view,
                const mrpt::Clock::time_point&) { callCount++; };

  std::thread loggerThread(
      [&]()
      {
        while (!stop)
        {
          log.logStr(mrpt::system::LVL_INFO, "tick");
        }
      });

  std::thread registerThread(
      [&]()
      {
        for (int i = 0; i < 200; i++)
        {
          log.logRegisterCallback(cb);
          log.logDeregisterCallback(cb);
        }
      });

  registerThread.join();
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  stop = true;
  loggerThread.join();

  SUCCEED();  // reaching here without crashing/TSan-flagging is the test
}
