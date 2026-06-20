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
#pragma once

/**
 * @file   WorkerThreadsPool.h
 * @brief  Simple thread pool for parallel task execution
 * @author Jose Luis Blanco Claraco
 * @date   Dec 6, 2018
 */

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <future>
#include <mutex>
#include <queue>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <vector>

namespace mrpt
{
/** \addtogroup mrpt_core_grp
 * @{ */

/**
 * @brief A simple and efficient thread pool for parallel task execution.
 *
 * This class manages a fixed number of worker threads that remain blocked
 * waiting for jobs to be assigned via enqueue(). The enqueue() method accepts
 * any callable object (function, lambda, functor) with arbitrary parameters
 * and returns a std::future<ReturnType> that can be used to:
 *   - Wait for task completion
 *   - Retrieve the function's return value
 *   - Catch any exceptions thrown by the task
 *
 * @par Queue Policies
 * When more tasks are assigned than available free threads, two policies exist:
 *   - @ref POLICY_FIFO: All jobs are enqueued and executed in order (default)
 *   - @ref POLICY_DROP_OLD: Older pending jobs are discarded when the queue
 *     exceeds the thread count. Note: running jobs are never aborted.
 *
 * @par Thread Safety
 * All public methods are thread-safe and can be called concurrently.
 *
 * @par Example Usage
 * @code
 * mrpt::WorkerThreadsPool pool(4);  // 4 worker threads
 *
 * // Enqueue a task and get a future
 * auto future = pool.enqueue([](int x) { return x * 2; }, 21);
 *
 * // Wait for result
 * int result = future.get();  // result == 42
 * @endcode
 *
 * @note Partly based on: https://github.com/progschj/ThreadPool (ZLib license)
 *
 * @see std::async, std::thread
 */
class WorkerThreadsPool
{
 public:
  /**
   * @brief Queue overflow policy when all threads are busy.
   */
  enum queue_policy_t : uint8_t
  {
    /** Default policy: *all* tasks are executed in FIFO order. No drops.
     *  Tasks will queue up indefinitely until workers become available. */
    POLICY_FIFO,

    /** If a task arrives and there are more pending tasks than worker
     *  threads, drop the oldest pending tasks until queue size equals
     *  thread count minus one. Running tasks are never aborted.
     *  Useful for real-time systems where old data becomes stale. */
    POLICY_DROP_OLD
  };

  /** @name Construction and Destruction
   * @{ */

  /**
   * @brief Default constructor. Creates a pool with no threads.
   *
   * Call resize() to add worker threads before enqueuing tasks.
   */
  WorkerThreadsPool() = default;

  /**
   * @brief Constructs a thread pool with the specified number of threads.
   *
   * @param[in] num_threads Number of worker threads to create. Should be > 0.
   * @param[in] policy Queue overflow policy (default: POLICY_FIFO)
   * @param[in] threadsName Base name for worker threads (for debugging)
   *
   * @note Thread names will be formatted as "${threadsName}[i]" where i is
   *       the thread index (0-based).
   */
  explicit WorkerThreadsPool(
      std::size_t num_threads,
      queue_policy_t policy = POLICY_FIFO,
      const std::string& threadsName = "WorkerThreadsPool") :
      policy_(policy)
  {
    resize(num_threads);
    name(threadsName);
  }

  /**
   * @brief Destructor. Stops all threads and waits for them to finish.
   *
   * @warning Any pending (not yet started) tasks will be discarded.
   *          Currently running tasks will complete before destruction.
   */
  ~WorkerThreadsPool() { clear(); }

  // Non-copyable
  WorkerThreadsPool(const WorkerThreadsPool&) = delete;
  WorkerThreadsPool& operator=(const WorkerThreadsPool&) = delete;
  WorkerThreadsPool(WorkerThreadsPool&&) = delete;
  WorkerThreadsPool& operator=(WorkerThreadsPool&&) = delete;

  /** @} */

  /** @name Thread Management
   * @{ */

  /**
   * @brief Adds worker threads to the pool.
   *
   * @param[in] num_threads Number of threads to add.
   *
   * @note This method adds threads to the existing pool. To replace all
   *       threads, call clear() first.
   */
  void resize(std::size_t num_threads);

  /**
   * @brief Returns the number of worker threads in the pool.
   * @return Current thread count.
   */
  [[nodiscard]] std::size_t size() const noexcept { return threads_.size(); }

  /**
   * @brief Stops all worker threads and clears the pool.
   *
   * This method:
   *   1. Signals all threads to stop
   *   2. Wakes up all waiting threads
   *   3. Waits for all threads to finish (joins them)
   *   4. Clears the thread container
   *
   * @warning Any pending tasks that haven't started will be discarded.
   *          A warning is printed to stderr if tasks are discarded.
   */
  void clear();

  /** @} */

  /** @name Task Management
   * @{ */

  /**
   * @brief Enqueues a task for execution by a worker thread.
   *
   * @tparam F Callable type (function, lambda, functor, etc.)
   * @tparam Args Argument types for the callable
   *
   * @param[in] f The callable to execute
   * @param[in] args Arguments to pass to the callable
   *
   * @return std::future that will hold the result of f(args...).
   *         Use future.get() to wait for and retrieve the result.
   *         Any exception thrown by f will be stored and re-thrown
   *         when get() is called.
   *
   * @throws std::runtime_error if called after clear() or during destruction.
   *
   * @note The returned future must not be discarded ([[nodiscard]]).
   *       If you don't need the result, store the future and let it
   *       destruct naturally, or call future.wait() to ensure completion.
   *
   * @par Example
   * @code
   * auto f1 = pool.enqueue([]{ return 42; });
   * auto f2 = pool.enqueue([](int a, int b){ return a + b; }, 1, 2);
   *
   * std::cout << f1.get() << ", " << f2.get();  // "42, 3"
   * @endcode
   */
  template <class F, class... Args>
  [[nodiscard]] auto enqueue(F&& f, Args&&... args)
      -> std::future<std::invoke_result_t<F, Args...>>;

  /**
   * @brief Returns the number of tasks waiting in the queue.
   *
   * This does NOT include tasks currently being executed by worker threads.
   *
   * @return Number of pending (queued but not started) tasks.
   */
  [[nodiscard]] std::size_t pendingTasks() const noexcept;

  /** @} */

  /** @name Thread Naming
   * @{ */

  /**
   * @brief Sets the base name for threads in this pool.
   *
   * Thread names are visible in debuggers, profilers, and system tools.
   * Individual threads will be named "${name}[i]" where i is the index.
   *
   * @param[in] name Base name for the threads (max ~15 chars on Linux)
   *
   * @note New in MRPT 2.1.5
   *
   * @par Example
   * @code
   * pool.name("MyPool");
   * // Threads will be named: MyPool[0], MyPool[1], ...
   * @endcode
   */
  void name(const std::string& name);

  /**
   * @brief Returns the base name of threads in this pool.
   * @return The thread name prefix set via name() or constructor.
   */
  [[nodiscard]] std::string name() const noexcept { return name_; }

  /** @} */

 private:
  /** @brief Worker thread main loop. Waits for and executes tasks. */
  void workerLoop() noexcept;

  std::vector<std::thread> threads_;
  std::atomic_bool do_stop_{false};
  mutable std::mutex queue_mutex_;  ///< mutable to allow const methods to lock
  std::condition_variable condition_;
  std::queue<std::function<void()>> tasks_;
  queue_policy_t policy_{POLICY_FIFO};
  std::string name_{"WorkerThreadsPool"};
};

// ============================================================================
//                          Template Implementation
// ============================================================================

template <class F, class... Args>
auto WorkerThreadsPool::enqueue(F&& f, Args&&... args)
    -> std::future<std::invoke_result_t<F, Args...>>
{
  using return_type = std::invoke_result_t<F, Args...>;

  // Create a packaged task wrapping the callable with its arguments
  auto task = std::make_shared<std::packaged_task<return_type()>>(
      std::bind(std::forward<F>(f), std::forward<Args>(args)...));

  std::future<return_type> result = task->get_future();

  {
    std::unique_lock<std::mutex> lock(queue_mutex_);

    // Don't allow enqueueing after stopping the pool
    if (do_stop_)
    {
      throw std::runtime_error("[WorkerThreadsPool] enqueue() called on stopped pool");
    }

    // Policy check: drop pending tasks if queue exceeds thread count
    if (policy_ == POLICY_DROP_OLD)
    {
      while (tasks_.size() >= threads_.size())
      {
        tasks_.pop();
      }
    }

    // Enqueue the new task
    tasks_.emplace([task = std::move(task)]() { (*task)(); });
  }

  condition_.notify_one();
  return result;
}

/** @} */  // end of mrpt_core_grp

}  // namespace mrpt