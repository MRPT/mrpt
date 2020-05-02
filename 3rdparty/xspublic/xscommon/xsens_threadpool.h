
//  Copyright (c) 2003-2019 Xsens Technologies B.V. or subsidiaries worldwide.
//  All rights reserved.
//  
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//  
//  1.	Redistributions of source code must retain the above copyright notice,
//  	this list of conditions, and the following disclaimer.
//  
//  2.	Redistributions in binary form must reproduce the above copyright notice,
//  	this list of conditions, and the following disclaimer in the documentation
//  	and/or other materials provided with the distribution.
//  
//  3.	Neither the names of the copyright holders nor the names of their contributors
//  	may be used to endorse or promote products derived from this software without
//  	specific prior written permission.
//  
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
//  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
//  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
//  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
//  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS 
//  SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES 
//  OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE 
//  ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.
//  

#ifndef XSENS_THREADPOOL_H
#define XSENS_THREADPOOL_H

#include "xsens_mutex.h"

#include <map>
#include <set>
#include <deque>
#include <list>
#include <memory>

namespace xsens {

int processorCount();

class PooledTask;
class ThreadPoolTask {
public:
	virtual bool exec() = 0;				//!< \returns True if the task completed or false to reschedule the task
	virtual unsigned int needToWaitFor();
	ThreadPoolTask() : m_container(nullptr) {}
	virtual ~ThreadPoolTask() {}
	virtual void onError() {}				//!< Callback for when an error occurred during task execution. Any implementation of this function should NEVER throw an exception.
	bool isCanceling() const;
	unsigned int taskId() const;

private:
	friend class ThreadPool;
	PooledTask* m_container;
};

class PooledThread;
class ThreadPool
{
public:
	typedef unsigned int TaskId;			//!< A type definition of a task ID

private:
	void reportTaskComplete(std::shared_ptr<PooledTask>);
	void reportTaskPaused(std::shared_ptr<PooledTask>);
	std::shared_ptr<PooledTask> getNextTask();
	friend class PooledThread;

	std::set<PooledThread*> m_threads;
	std::deque<std::shared_ptr<PooledTask>> m_tasks;
	std::map<TaskId, std::shared_ptr<PooledTask>> m_tasksSearch;
	std::map<TaskId, std::shared_ptr<PooledTask>> m_executing;
	std::map<TaskId, std::shared_ptr<PooledTask>> m_delaying;
	Mutex m_safe;
	TaskId m_nextId;
	bool m_suspended;
	volatile std::atomic_bool m_terminating;

	std::shared_ptr<PooledTask> findTask(TaskId id);

protected:
	ThreadPool();
	~ThreadPool();

public:
	TaskId addTask(ThreadPoolTask* task, TaskId afterId = 0);
	unsigned int count();
	void setPoolSize(unsigned int poolsize);
	unsigned int poolSize() const;
	bool doesTaskExist(TaskId id);
	void cancelTask(TaskId id, bool wait = true);
	void waitForCompletion(TaskId id);
	void suspend(bool wait = false) noexcept;
	void resume();
	unsigned int executedCount(unsigned int thread) const;
	unsigned int completedCount(unsigned int thread) const;
	unsigned int failedCount(unsigned int thread) const;
	XsThreadId taskThreadId(TaskId id);

	static ThreadPool* instance();
	static void destroy();
	static void setPool(ThreadPool* pool);
};

class TaskCompletionWaiter : public ThreadPoolTask
{
public:
	virtual bool exec();
	virtual unsigned int needToWaitFor();

	TaskCompletionWaiter(ThreadPool* pool = ThreadPool::instance());
	TaskCompletionWaiter(const std::list<unsigned int>& waitlist, ThreadPool* pool = ThreadPool::instance());
	virtual ~TaskCompletionWaiter();
	void addWaitId(unsigned int id);

private:
	ThreadPool* m_pool;
	std::list<unsigned int> m_waitList;
};

/*! \brief Task that will delete its object parameter
	\details This class can be used to schedule a delete of a single object in the ThreadPool.
	Normally you don't need to use this class directly, but can use the \ref deleteThreaded function.
	\tparam T The type of the object to delete
*/
template <typename T>
class ThreadPoolObjectDeleter : public ThreadPoolTask
{
	T* m_object;
	ThreadPoolObjectDeleter(ThreadPoolObjectDeleter const&) = delete;
	ThreadPoolObjectDeleter const& operator = (ThreadPoolObjectDeleter const&) = delete;
public:
	/*! \brief Constructor, creates the task to delete object a obj, but doesn't schedule it.
	*/
	ThreadPoolObjectDeleter(T* obj) : m_object(obj) {}
	/*! \brief deletes the managed object */
	bool exec() override
	{
		delete m_object;
		return true;
	}
};

/*! \brief Delete \a obj in the ThreadPool.
	\details This function will schedule a task to be executed by the ThreadPool which will delete obj
	\param obj The object to be deleted
	\tparam T The type of the object to delete. Typically not required as the compiler should detect it automatically.
	\return The task ID of the deletion task
	\relates ThreadPoolObjectDeleter
*/
template <typename T>
inline static ThreadPool::TaskId deleteThreaded(T* obj)
{
	return ThreadPool::instance()->addTask(new ThreadPoolObjectDeleter<T>(obj));
}

} // namespace xsens

#endif
