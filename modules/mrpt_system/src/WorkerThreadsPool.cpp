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

#include <mrpt/core/exceptions.h>
#include <mrpt/system/WorkerThreadsPool.h>

#include <iostream>

using namespace mrpt::system;

void WorkerThreadsPool::clear()
{
  {
    std::unique_lock<std::mutex> lock(queue_mutex_);
    do_stop_ = true;
  }
  condition_.notify_all();

  if (!tasks_.empty())
  {
    std::cerr << "[threadPool] Warning: clear() called (probably from a "
                 "dtor) while having "
              << tasks_.size() << " pending tasks. Aborting them.\n";
  }

  for (auto& t : threads_)
  {
    if (t.joinable())
    {
      t.join();
    }
  }
  threads_.clear();
}

std::size_t WorkerThreadsPool::pendingTasks() const noexcept { return tasks_.size(); }

void WorkerThreadsPool::resize(std::size_t num_threads)
{
  for (std::size_t i = 0; i < num_threads; ++i)
  {
    threads_.emplace_back(
        [this]
        {
          for (;;)
          {
            try
            {
              std::function<void()> task;
              {
                std::unique_lock<std::mutex> lock(queue_mutex_);
                condition_.wait(lock, [this] { return do_stop_ || !tasks_.empty(); });
                if (do_stop_)
                {
                  return;
                }
                task = std::move(tasks_.front());
                tasks_.pop();
              }
              // Execute:
              task();
            }
            catch (std::exception& e)
            {
              std::cerr << "[thread-pool] Exception:\n" << mrpt::exception_to_str(e) << "\n";
            }
          }
        });
  }
}
