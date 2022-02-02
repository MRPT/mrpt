/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

/**
 * @file   WorkerThreadsPool.h
 * @brief  Simple thread pool
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
#include <thread>
#include <vector>

namespace mrpt
{
/** \addtogroup mrpt_core_grp
 * @{ */

/** A thread pool: it defines a fixed number of threads, which will remain
 *  blocked waiting for jobs to be assigned, via WorkerThreadsPool::enqueue(),
 *  which accepts any function-like object with arbitrary parameters and
 *  returns a std::future<ReturnType> which can be used to wait and/or retrieve
 *  the function output at any moment in time afterwards.
 *
 *  In case of more tasks assigned than available free threads, two policies
 *  exist:
 *  - WorkerThreadsPool::POLICY_FIFO: All jobs are enqueued and wait for it turn
 *    to be executed.
 *  - WorkerThreadsPool::POLICY_DROP_OLD: Old jobs in the waiting queue are
 *    discarded. Note that running jobs are never aborted.
 *
 * \note Partly based on: https://github.com/progschj/ThreadPool (ZLib license)
 *
 * \note (New in MRPT 2.1.0)
 */
class WorkerThreadsPool
{
   public:
	enum queue_policy_t : uint8_t
	{
		/** Default policy: *all* tasks are executed in FIFO order. No drops. */
		POLICY_FIFO,
		/** If a task arrives and there are more pending tasks than worker
		   threads, drop previous tasks. */
		POLICY_DROP_OLD
	};

	WorkerThreadsPool() = default;
	WorkerThreadsPool(
		std::size_t num_threads, queue_policy_t p = POLICY_FIFO,
		const std::string& threadsName = "WorkerThreadsPool")
		: policy_(p)
	{
		resize(num_threads);
		name(threadsName);
	}
	~WorkerThreadsPool() { clear(); }
	void resize(std::size_t num_threads);
	/** Stops and deletes all worker threads */
	void clear();

	/** Enqueue one new working item, to be executed by threads when any is
	 * available. */
	template <class F, class... Args>
	[[nodiscard]] auto enqueue(F&& f, Args&&... args)
		-> std::future<typename std::result_of<F(Args...)>::type>;

	/** Returns the number of enqueued tasks, currently waiting for a free
	 * working thread to process them.  */
	std::size_t pendingTasks() const noexcept;

	/** Sets the private thread names of threads in this pool.
	 * Names can be seen from debuggers, profilers, etc. and will follow
	 * the format `${name}[i]` with `${name}` the value supplied in this method
	 * \note (Method new in MRPT 2.1.5)
	 */
	void name(const std::string& name);

	/** Returns the base name of threads in this pool */
	std::string name() const { return name_; }

   private:
	std::vector<std::thread> threads_;
	std::atomic_bool do_stop_{false};
	std::mutex queue_mutex_;
	std::condition_variable condition_;
	std::queue<std::function<void()>> tasks_;
	queue_policy_t policy_{POLICY_FIFO};
	std::string name_{"WorkerThreadsPool"};
};

template <class F, class... Args>
auto WorkerThreadsPool::enqueue(F&& f, Args&&... args)
	-> std::future<typename std::result_of<F(Args...)>::type>
{
	using return_type = typename std::result_of<F(Args...)>::type;

	auto task = std::make_shared<std::packaged_task<return_type()>>(
		std::bind(std::forward<F>(f), std::forward<Args>(args)...));

	std::future<return_type> res = task->get_future();
	{
		std::unique_lock<std::mutex> lock(queue_mutex_);

		// don't allow enqueueing after stopping the pool
		if (do_stop_) throw std::runtime_error("enqueue on stopped ThreadPool");

		// policy check: drop pending tasks if we have more tasks than threads
		if (policy_ == POLICY_DROP_OLD)
		{
			while (tasks_.size() >= threads_.size())
			{
				tasks_.pop();
			}
		}

		// Enqeue the new task:
		tasks_.emplace([task]() { (*task)(); });
	}
	condition_.notify_one();
	return res;
}

/** @} */
}  // namespace mrpt
