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
#include <mrpt/imgui/AsyncTask.h>

#include <chrono>
#include <stdexcept>
#include <thread>

using mrpt::imgui::AsyncTask;
using mrpt::imgui::AsyncTaskContext;

namespace
{
template <typename T>
T waitFor(AsyncTask<T>& task)
{
  for (int i = 0; i < 1000; i++)
  {
    if (auto r = task.poll(); r)
    {
      return *r;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  throw std::runtime_error("timeout");
}
}  // namespace

TEST(AsyncTask, ReturnsResultOnce)
{
  AsyncTask<int> task;
  EXPECT_FALSE(task.isRunning());
  EXPECT_FALSE(task.poll().has_value());

  task.start([]() { return 42; });
  EXPECT_TRUE(task.isRunning());
  EXPECT_EQ(task.generation(), 1u);
  EXPECT_EQ(waitFor(task), 42);
  EXPECT_FALSE(task.isRunning());
  EXPECT_FALSE(task.poll().has_value());  // only once
}

TEST(AsyncTask, ProgressAndCancel)
{
  AsyncTask<int> task;
  std::atomic<bool> sawCancel{false};
  task.start(
      [&](AsyncTaskContext& ctx)
      {
        ctx.setProgress(0.5, "halfway");
        while (!ctx.cancelRequested())
        {
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        sawCancel = true;
        return 1;
      });
  for (int i = 0; i < 500 && task.progress() < 0.5; i++)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  EXPECT_DOUBLE_EQ(task.progress(), 0.5);
  EXPECT_EQ(task.progressMessage(), "halfway");

  task.cancel();
  EXPECT_FALSE(task.isRunning());
  EXPECT_FALSE(task.poll().has_value());
  for (int i = 0; i < 500 && !sawCancel; i++)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  EXPECT_TRUE(sawCancel);
}

TEST(AsyncTask, NewStartSupersedesRunningOne)
{
  AsyncTask<int> task;
  task.start(
      [](AsyncTaskContext& ctx)
      {
        while (!ctx.cancelRequested())
        {
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        return 1;
      });
  task.start([]() { return 2; });
  EXPECT_EQ(task.generation(), 2u);
  EXPECT_EQ(waitFor(task), 2);
}

TEST(AsyncTask, ExceptionIsRethrownByPoll)
{
  AsyncTask<int> task;
  task.start([]() -> int { throw std::runtime_error("boom"); });
  bool thrown = false;
  for (int i = 0; i < 1000 && !thrown; i++)
  {
    try
    {
      (void)task.poll();
    }
    catch (const std::runtime_error&)
    {
      thrown = true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  EXPECT_TRUE(thrown);
}

TEST(AsyncTask, CancelAndWait)
{
  AsyncTask<int> task;
  std::atomic<bool> finished{false};
  task.start(
      [&](AsyncTaskContext& ctx)
      {
        while (!ctx.cancelRequested())
        {
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        finished = true;
        return 1;
      });
  task.cancelAndWait();
  EXPECT_TRUE(finished);  // the worker has returned already
  EXPECT_FALSE(task.isRunning());
  EXPECT_FALSE(task.poll().has_value());
  task.cancelAndWait();  // no-op
}
