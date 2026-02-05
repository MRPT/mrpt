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

/**
 * @file   WorkerThreadsPool.cpp
 * @brief  Simple thread pool for parallel task execution
 * @author Jose Luis Blanco Claraco
 * @date   Dec 6, 2018
 */

#include <mrpt/core/WorkerThreadsPool.h>
#include <mrpt/core/config.h>
#include <mrpt/core/exceptions.h>

#include <iostream>

// For SetThreadDescription() on Windows
#if defined(_WIN32)
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif

namespace mrpt
{

void WorkerThreadsPool::clear()
{
  {
    std::scoped_lock lock(queue_mutex_);
    do_stop_ = true;
  }
  condition_.notify_all();

  // Warn about discarded tasks (but only if there are any)
  if (const auto pending = tasks_.size(); pending > 0)
  {
    std::cerr << "[WorkerThreadsPool name='" << name_
              << "'] Warning: clear() called (probably from destructor) "
                 "while having "
              << pending << " pending task(s). Discarding them.\n";
  }

  // Join all threads
  for (auto& t : threads_)
  {
    if (t.joinable())
    {
      t.join();
    }
  }
  threads_.clear();
}

std::size_t WorkerThreadsPool::pendingTasks() const noexcept
{
  std::scoped_lock lock(queue_mutex_);
  return tasks_.size();
}

void WorkerThreadsPool::resize(std::size_t num_threads)
{
  threads_.reserve(threads_.size() + num_threads);

  for (std::size_t i = 0; i < num_threads; ++i)
  {
    threads_.emplace_back([this] { workerLoop(); });
  }
}

void WorkerThreadsPool::workerLoop() noexcept
{
  for (;;)
  {
    try
    {
      std::function<void()> task;

      // Wait for a task or stop signal
      {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        condition_.wait(lock, [this] { return do_stop_ || !tasks_.empty(); });

        if (do_stop_ && tasks_.empty())
        {
          return;
        }

        task = std::move(tasks_.front());
        tasks_.pop();
      }

      // Execute the task outside the lock
      task();
    }
    catch (const std::exception& e)
    {
      std::cerr << "[WorkerThreadsPool name='" << name_
                << "'] Unhandled exception in worker thread:\n"
                << mrpt::exception_to_str(e) << "\n";
    }
    catch (...)
    {
      std::cerr << "[WorkerThreadsPool name='" << name_
                << "'] Unknown exception in worker thread.\n";
    }
  }
}

// Platform-specific thread naming implementation
namespace
{
void setThreadName(
    [[maybe_unused]] const std::string& name, [[maybe_unused]] std::thread& theThread)
{
#if defined(MRPT_OS_WINDOWS) && !defined(__MINGW32_MAJOR_VERSION)
  wchar_t wName[50];
  std::mbstowcs(wName, name.c_str(), sizeof(wName) / sizeof(wName[0]));
  SetThreadDescription(theThread.native_handle(), wName);
#elif defined(MRPT_OS_LINUX) && !MRPT_IN_EMSCRIPTEN
  auto handle = theThread.native_handle();
  // Note: pthread_setname_np has a 16-char limit (including null terminator)
  pthread_setname_np(handle, name.substr(0, 15).c_str());
#endif
}
}  // namespace

void WorkerThreadsPool::name(const std::string& name)
{
  name_ = name;

  for (std::size_t i = 0; i < threads_.size(); ++i)
  {
    const std::string threadName = name + "[" + std::to_string(i) + "]";
    setThreadName(threadName, threads_[i]);
  }
}

}  // namespace mrpt