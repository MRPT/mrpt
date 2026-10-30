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

namespace
{
// Identity-based deregistration only works for callbacks stored as actual
// function pointers (free functions / static members / non-capturing
// lambdas *explicitly* decayed to a function pointer): logDeregisterCallback()
// compares std::function::target<FnPtr>(), which is only non-null when the
// std::function was constructed directly from a value of that exact pointer
// type (assigning a lambda closure directly does NOT count, even if the
// lambda is non-capturing).
int g_callCountA = 0;
int g_callCountB = 0;

void CallbackA(
    std::string_view,
    const mrpt::system::VerbosityLevel,
    std::string_view,
    const mrpt::Clock::time_point)
{
  g_callCountA++;
}

void CallbackB(
    std::string_view,
    const mrpt::system::VerbosityLevel,
    std::string_view,
    const mrpt::Clock::time_point)
{
  g_callCountB++;
}
}  // namespace

TEST(COutputLogger, RegisterDeregisterCallback)
{
  mrpt::system::COutputLogger log;
  log.setVerbosityLevel(mrpt::system::LVL_DEBUG);

  g_callCountA = g_callCountB = 0;
  int& callCountA = g_callCountA;
  int& callCountB = g_callCountB;
  // Two distinct function pointers, so that logDeregisterCallback() removing
  // "a" cannot spuriously also match "b".
  mrpt::system::output_logger_callback_t cbA = &CallbackA;
  mrpt::system::output_logger_callback_t cbB = &CallbackB;

  log.logRegisterCallback(cbA);
  log.logRegisterCallback(cbB);
  log.logStr(mrpt::system::LVL_INFO, "hello");
  EXPECT_EQ(callCountA, 1);
  EXPECT_EQ(callCountB, 1);

  // Deregistering "a" must stop only "a" from firing, "b" must keep firing.
  EXPECT_TRUE(log.logDeregisterCallback(cbA));
  log.logStr(mrpt::system::LVL_INFO, "world");
  EXPECT_EQ(callCountA, 1);  // no further calls after deregistering
  EXPECT_EQ(callCountB, 2);  // still active

  EXPECT_FALSE(log.logDeregisterCallback(cbA));  // already removed

  EXPECT_TRUE(log.logDeregisterCallback(cbB));
  log.logStr(mrpt::system::LVL_INFO, "!");
  EXPECT_EQ(callCountB, 2);  // no further calls after deregistering
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

  std::atomic<bool> start{false};
  std::atomic<bool> stop{false};
  std::atomic<int> callCount{0};

  auto cb = [&](std::string_view, const mrpt::system::VerbosityLevel, std::string_view,
                const mrpt::Clock::time_point&) { callCount++; };

  std::thread loggerThread(
      [&]()
      {
        while (!start)
        {
        }
        while (!stop)
        {
          log.logStr(mrpt::system::LVL_INFO, "tick");
        }
      });

  std::thread registerThread(
      [&]()
      {
        while (!start)
        {
        }
        for (int i = 0; i < 5000; i++)
        {
          const auto id = log.logRegisterCallback(cb);
          log.logDeregisterCallback(id);
        }
      });

  start = true;
  registerThread.join();
  stop = true;
  loggerThread.join();

  SUCCEED();  // reaching here without crashing/TSan-flagging is the test
}

TEST(COutputLogger, DeregisterCallbackById)
{
  mrpt::system::COutputLogger log;
  log.logging_enable_console_output = false;

  int countA = 0;
  int countB = 0;
  // Capturing lambdas: only removable by identifier.
  const auto idA =
      log.logRegisterCallback([&countA](
                                  std::string_view, const mrpt::system::VerbosityLevel,
                                  std::string_view, const mrpt::Clock::time_point) { countA++; });
  const auto idB =
      log.logRegisterCallback([&countB](
                                  std::string_view, const mrpt::system::VerbosityLevel,
                                  std::string_view, const mrpt::Clock::time_point) { countB++; });
  EXPECT_NE(idA, 0U);
  EXPECT_NE(idB, 0U);
  EXPECT_NE(idA, idB);

  log.logStr(mrpt::system::LVL_INFO, "hello");
  EXPECT_EQ(countA, 1);
  EXPECT_EQ(countB, 1);

  EXPECT_TRUE(log.logDeregisterCallback(idA));
  log.logStr(mrpt::system::LVL_INFO, "world");
  EXPECT_EQ(countA, 1);
  EXPECT_EQ(countB, 2);

  EXPECT_FALSE(log.logDeregisterCallback(idA));               // already removed
  EXPECT_FALSE(log.logDeregisterCallback(idA + idB + 1000));  // never existed

  EXPECT_TRUE(log.logDeregisterCallback(idB));
  log.logStr(mrpt::system::LVL_INFO, "!");
  EXPECT_EQ(countB, 2);

  // Identifiers are not reused:
  const auto idC = log.logRegisterCallback([](std::string_view, const mrpt::system::VerbosityLevel,
                                              std::string_view, const mrpt::Clock::time_point) {});
  EXPECT_NE(idC, idA);
  EXPECT_NE(idC, idB);
}

// A callback that stays registered must get every message, while other
// callbacks are added and removed from another thread, and the verbosity
// level for callbacks is changed from yet another one. Run under TSan to
// catch data races.
TEST(COutputLogger, ConcurrentCallbackChangesAndLog)
{
  mrpt::system::COutputLogger log;
  log.logging_enable_console_output = false;

  std::atomic<int> permanentCount{0};
  log.logRegisterCallback([&permanentCount](
                              std::string_view, const mrpt::system::VerbosityLevel,
                              std::string_view, const mrpt::Clock::time_point)
                          { permanentCount++; });

  constexpr int NUM_MESSAGES = 20000;
  std::atomic<bool> stop{false};

  std::thread loggerThread(
      [&]()
      {
        for (int i = 0; i < NUM_MESSAGES; i++)
        {
          log.logStr(mrpt::system::LVL_WARN, "tick");
        }
        stop = true;
      });

  std::thread registerThread(
      [&]()
      {
        while (!stop)
        {
          const auto id =
              log.logRegisterCallback([](std::string_view, const mrpt::system::VerbosityLevel,
                                         std::string_view, const mrpt::Clock::time_point) {});
          log.logDeregisterCallback(id);
        }
      });

  // WARN messages pass both thresholds, so the count below is exact:
  std::thread levelThread(
      [&]()
      {
        while (!stop)
        {
          log.setVerbosityLevelForCallbacks(mrpt::system::LVL_DEBUG);
          log.setVerbosityLevelForCallbacks(mrpt::system::LVL_INFO);
        }
      });

  loggerThread.join();
  registerThread.join();
  levelThread.join();

  EXPECT_EQ(permanentCount.load(), NUM_MESSAGES);
}
