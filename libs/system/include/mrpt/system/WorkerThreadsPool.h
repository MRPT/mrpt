/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <future>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

namespace mrpt::system
{
/**
 * @brief A simple thread pool
 *
 * \note Partly based on: https://github.com/progschj/ThreadPool (ZLib license)
 */
class WorkerThreadsPool
{
   public:
	enum queue_policy_t : uint8_t
	{
		/** Default policy: all tasks are executed in FIFO order */
		POLICY_FIFO,
		/** If a task arrives and there are more pending tasks than worker
		   threads, drop previous tasks. */
		POLICY_DROP_OLD
	};

	WorkerThreadsPool() = default;
	WorkerThreadsPool(std::size_t num_threads, queue_policy_t p = POLICY_FIFO)
		: policy_(p)
	{
		resize(num_threads);
	}
	~WorkerThreadsPool() { clear(); }
	void resize(std::size_t num_threads);
	void clear();  //!< Stops all working jobs

	/** Enqueue one new working item, to be executed by threads when any is
	 * available. */
	template <class F, class... Args>
	auto enqueue(F&& f, Args&&... args)
		-> std::future<typename std::result_of<F(Args...)>::type>;

	/** Returns the number of enqueued tasks, currently waiting for a free
	 * working thread to process them.  */
	std::size_t pendingTasks() const noexcept;

   private:
	std::vector<std::thread> threads_;
	std::atomic_bool do_stop_{false};
	std::mutex queue_mutex_;
	std::condition_variable condition_;
	std::queue<std::function<void()>> tasks_;
	queue_policy_t policy_{POLICY_FIFO};
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
}  // namespace mrpt::system
