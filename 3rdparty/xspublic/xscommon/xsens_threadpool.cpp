
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

#include "xsens_threadpool.h"
#ifndef _MSC_VER
	#include <unistd.h>
	#include <cxxabi.h>
#endif
#ifdef XSENS_DEBUG
	#include <typeinfo>
#endif

#include <vector>
#include <xstypes/xsexception.h>
#include "threading.h"
#include <atomic>

// Enable this if you want to disable multithreading
//#define ADD_TASK_EXECUTE_NOW

namespace xsens {
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////

typedef std::map<unsigned int, std::shared_ptr<PooledTask>> TaskSet;
typedef std::set<PooledThread*> ThreadSet;
typedef std::vector<std::shared_ptr<PooledTask>> TaskList;

/*! \brief Returns the number of processor cores in the current system
*/
int processorCount()
{
#if defined(_WIN32)
	SYSTEM_INFO sysinfo;
	::GetSystemInfo( &sysinfo );
	return (int) sysinfo.dwNumberOfProcessors;
#else
	return sysconf(_SC_NPROCESSORS_ONLN);
	// see http://stackoverflow.com/questions/150355/programmatically-find-the-number-of-cores-on-a-machine
	// for OS-specific code
#endif
}

/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////

/*! \class ThreadPoolTask
	\brief A generic task implementation for the thread pool
*/

/*! \brief This function gets called by PooledThread when the exec() function returns false to determine if we should wait for a specific task
	\details The default implementation returns 0 (no id to wait for)
	\returns False to determine if we should wait for a specific task
*/
unsigned int ThreadPoolTask::needToWaitFor()
{
	return 0;
}

/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////

/*! \brief A class that contains a task and some administrative stuff
*/
class PooledTask {
public:
	ThreadPoolTask* m_task;		//!< The task that is to be executed
	unsigned int m_id;			//!< The id that was assigned to the task by the ThreadPool
	std::vector<std::shared_ptr<PooledTask>> m_dependentTasks;	//!< A list of tasks that are waiting for this task to complete
	XsThreadId m_threadId;
	volatile std::atomic<bool> m_canceling;

	PooledTask()
		: m_task(nullptr)
		, m_id(0)
		, m_threadId(0)
		, m_canceling(false)
		, m_completed(false)
		, m_completedMutex()
		, m_completedCondition(m_completedMutex)
	{
	}

	~PooledTask()
	{
		signalCompleted();
		delete m_task;
	}

	bool waitForCompletion(uint32_t timeout = UINT32_MAX)
	{
		Lock locker(&m_completedMutex);
		if (!m_completed)
		{
			// This may look like a dead-lock situation with signalCompleted(), but it isn't
			if (timeout == UINT32_MAX)
				m_completedCondition.wait();
			else
				m_completedCondition.wait(timeout);
		}
		return m_completed;
	}

	void signalCompleted() noexcept
	{
		Lock locker(&m_completedMutex);
		if (!m_completed)
		{
			m_completed = true;
			locker.unlock();
			m_completedCondition.broadcast();
		}
	}

private:
	volatile std::atomic_bool m_completed;
	Mutex m_completedMutex;
	WaitCondition m_completedCondition;
};

/*! \brief Returns true if the task has been told to cancel itself */
bool ThreadPoolTask::isCanceling() const
{
	if (m_container)
		return m_container->m_canceling;
	return false;
}

/*! \brief Returns the task ID of the task or 0 if it doesn't have a proper ID (yet) */
unsigned int ThreadPoolTask::taskId() const
{
	if (m_container)
		return m_container->m_id;
	return 0;
}

/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////

/*! \brief A class that contains a thread that runs in a ThreadPool to execute tasks
	\details These threads are created by the ThreadPool.
*/
class PooledThread : public StandardThread
{
public:
	PooledThread(ThreadPool* pool);
	~PooledThread();
	bool isIdle() const;
	bool isBusy() const;
	unsigned int executedCount() const;
	unsigned int completedCount() const;
	unsigned int failedCount() const;

protected:
	ThreadPool* m_pool;			//!< The pool that contains this thread
	std::shared_ptr<PooledTask> m_task;			//!< The task that is currently being executed or NULL if the thread is idle
	unsigned int m_executed;	//!< The number of tasks that this thread has executed s o far, including incomplete tasks
	unsigned int m_completed;	//!< The number of tasks that this thread has completed so far, excluding incomplete tasks
	unsigned int m_failed;		//!< The number of tasks that this thread has failed to complete so far due to an exception

	virtual int32_t innerFunction(void);
};

/*! \brief Constructor */
PooledThread::PooledThread(ThreadPool* pool)
	: StandardThread()
	, m_pool(pool)
	, m_task(nullptr)
	, m_executed(0)
	, m_completed(0)
	, m_failed(0)
{
}

/*! \brief Destructor */
PooledThread::~PooledThread()
{
	stopThread();
	m_pool = nullptr;
}

/*! \brief The inner function of the pooled thread.

	The function will do tasks until the ThreadPool no longer supplies any tasks, at which point
	it will return with a sleep time of 1ms.
*/
int32_t PooledThread::innerFunction(void)
{
	if (!m_task)
		m_task = m_pool->getNextTask();

	while (m_task && !isTerminating())
	{
		m_task->m_threadId = getThreadId();
		bool complete = false;
		try
		{
			if (m_task->m_task->exec())
			{
				++m_completed;
				complete = true;
			}
			else if (m_task->m_canceling)
			{
				++m_failed;
				complete = true;
			}
		}
#ifdef XSENS_DEBUG
		catch (std::exception &e)
		{
			const std::type_info &et = typeid(e);
#ifdef __GNUC__
			int status;
			char *realname = abi::__cxa_demangle(et.name(), 0, 0, &status);
			fprintf(stderr, "ThreadPool: Caught an unhandled %s(\"%s\")\n", realname, e.what());
			free(realname);
#else
			fprintf(stderr, "ThreadPool: Caught an unhandled %s: %s\n", et.name(), e.what());
#endif
			complete = true;
		}
#endif
		catch(...)
		{
#ifdef XSENS_DEBUG
			fprintf(stderr, "ThreadPool: Caught an unhandled unknown exception\n");
#endif
			m_task->m_task->onError();
			++m_failed;
			complete = true;
		}

		++m_executed;
		if (complete)
			m_pool->reportTaskComplete(m_task);
		else
		{
			m_task->m_threadId = 0;
			m_pool->reportTaskPaused(m_task);
		}

		m_task = m_pool->getNextTask();
	}
	return 1;
}

/*! \brief Return whether the thread is currently executing a task (false) or not (true)
*/
bool PooledThread::isIdle() const
{
	return (m_task == nullptr);
}

/*! \brief Return whether the thread is currently executing a task (true) or not (false)
*/
bool PooledThread::isBusy() const
{
	return !isIdle();
}

/*! \brief Return the number of tasks successfully or partially executed by the thread
*/
unsigned int PooledThread::executedCount() const
{
	return m_executed;
}

/*! \brief Return the number of tasks successfully executed by the thread
*/
unsigned int PooledThread::completedCount() const
{
	return m_completed;
}

/*! \brief Return the number of tasks that failed to execute properly
*/
unsigned int PooledThread::failedCount() const
{
	return m_failed;
}

/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////

/*! \class ThreadPool
	\brief This class creates and maintains a number of threads that can execute finite-length tasks
	\note See the test cases for examples on how to use the class
*/

/*! \brief Construct a threadpool with a number of threads equal to the number of cores on the PC
*/
ThreadPool::ThreadPool()
	: m_nextId(1)
	, m_suspended(false)
	, m_terminating(false)
{
	setPoolSize(0);
}

/*! \brief Destructor, clears any pending tasks and destroys the threads
*/
ThreadPool::~ThreadPool()
{
	m_terminating = true;
	suspend(true);

	// all threads should now be idle
	m_tasks.clear();
	m_tasksSearch.clear();
	m_executing.clear();
	m_delaying.clear();

	try
	{
		for (ThreadSet::iterator it = m_threads.begin(); it != m_threads.end(); ++it)
			delete *it;
	}
	catch(...)
	{
		// nothing much we can do about this...
	}
}

/*! \brief Add a task to be executed by the threadpool
	\param task The task object whose exec() function will be called
	\param afterId When not 0, the task will not be started until the task with the given id has completed

	\returns The id of the task that was added or 0 if the task could not be added for some reason.
*/
ThreadPool::TaskId ThreadPool::addTask(ThreadPoolTask* task, ThreadPool::TaskId afterId)
{
	assert(!m_terminating);
	std::shared_ptr<PooledTask> tmp(new PooledTask);
	tmp->m_task = task;
	Lock safety(&m_safe);
	tmp->m_id = m_nextId;
	task->m_container = tmp.get();

	// determine next ID
	if (!++m_nextId)
		m_nextId = 1;

#ifndef ADD_TASK_EXECUTE_NOW
	if (afterId)
	{
		std::shared_ptr<PooledTask> after = findTask(afterId);
		if (after)
		{
			after->m_dependentTasks.push_back(tmp);
			m_delaying[tmp->m_id] = tmp;
			return tmp->m_id;
		}
	}

	m_tasks.push_back(tmp);
	m_tasksSearch[tmp->m_id] = tmp;
#else
	task->exec();
	task->signalCompleted();
#endif

	return tmp->m_id;
}

/*! \brief Return the number of tasks that are currently in the queue or being executed
*/
unsigned int ThreadPool::count()
{
	Lock safety(&m_safe);
	return (unsigned int) (m_tasks.size() + m_delaying.size() + m_executing.size());
}

/*! \brief Set the number of threads in the ThreadPool
	\param poolsize When 0 or less two threads will be created for each processor core in the system (with a minimum of 4 threads), otherwise the desired number of threads will be created
*/
void ThreadPool::setPoolSize(unsigned int poolsize)
{
	if (poolsize == 0)
	{
		//int pc = processorCount();
		//poolsize = ::std::max(4, pc*2);
		poolsize = 12;	// fixed count to prevent thread starvation on low-core PCs and overthreading on high-core PCs
	}

	bool wasSuspended = m_suspended;
	suspend(poolsize < m_threads.size());
	Lock safety(&m_safe);

	// reduce size if pool is too large
	if (poolsize < m_threads.size())
	{
		while (poolsize < m_threads.size())
		{
			delete *m_threads.begin();
			m_threads.erase(m_threads.begin());
		}
	}

	// increase size if pool is too small
	for (unsigned int i = (unsigned int) m_threads.size(); i < poolsize; ++i)
	{
		PooledThread* t = new PooledThread(this);
		m_threads.insert(t);
#ifdef XSENS_DEBUG
		char bufje[64];
		sprintf(bufje, "Pooled Thread %p", t);
#else
		const char bufje[] = "Pooled Thread";
#endif
		if (!t->startThread(bufje))
		{
			m_threads.erase(m_threads.find(t));
			delete t;
			throw XsException(XRV_ERROR, "Could not start thread for ThreadPool");
		}
	}

	if (!wasSuspended)
		resume();
}

/*! \brief Return the number of threads in the pool */
unsigned int ThreadPool::poolSize() const
{
	return (unsigned int) m_threads.size();
}

/*! \brief Find a task with the supplied \a id
*/
std::shared_ptr<PooledTask> ThreadPool::findTask(ThreadPool::TaskId id)
{
	Lock safety(&m_safe);
	TaskSet::iterator it = m_executing.find(id);
	if (it != m_executing.end())
		return it->second;

	it = m_delaying.find(id);
	if (it != m_delaying.end())
		return it->second;

	it = m_tasksSearch.find(id);
	if (it != m_tasksSearch.end())
		return it->second;

	return std::shared_ptr<PooledTask>();
}

/*! \brief Find an XsThread with the specified \a id
*/
XsThreadId ThreadPool::taskThreadId(TaskId id)
{
	Lock safety(&m_safe);
	TaskSet::iterator it = m_executing.find(id);
	if (it != m_executing.end())
		return it->second->m_threadId;
	return 0;
}

/*! \brief Check if a task with the supplied \a id exists
*/
bool ThreadPool::doesTaskExist(ThreadPool::TaskId id)
{
	return findTask(id) != nullptr;
}

/*! \brief Remove the task with the supplied \a id if it exists, waits for the task to be finished
*/
void ThreadPool::cancelTask(ThreadPool::TaskId id, bool wait)
{
	Lock safety(&m_safe);
	TaskSet::iterator it = m_executing.find(id);
	if (it != m_executing.end())
	{
		it->second->m_canceling = true;
		safety.unlock();
		if (wait)
			waitForCompletion(id);
		return;
	}

	it = m_delaying.find(id);
	if (it != m_delaying.end())
		m_delaying.erase(it);

	it = m_tasksSearch.find(id);
	if (it != m_tasksSearch.end())
		m_tasksSearch.erase(it);
}

/*! \brief Wait for the task with the given ID to complete
*/
void ThreadPool::waitForCompletion(ThreadPool::TaskId id)
{
	std::shared_ptr<PooledTask> task = findTask(id);
	if (task != nullptr)
		task->waitForCompletion();
}

/*! \brief Called by PooledThread to notify the ThreadPool that a task was completed
	\note After this call, the supplied \a task is invalid
*/
void ThreadPool::reportTaskComplete(std::shared_ptr<PooledTask> task)
{
	Lock safety(&m_safe);

	// remove task from 'executing' list
	TaskSet::iterator it = m_executing.find(task->m_id);
	if (it != m_executing.end())
		m_executing.erase(it);

	// notify dependent tasks that their dependency has been fulfilled
	for (TaskList::iterator dep = task->m_dependentTasks.begin(); dep != task->m_dependentTasks.end(); ++dep)
	{
		it = m_delaying.find((*dep)->m_id);
		if (it != m_delaying.end())
		{
			m_tasks.push_back(it->second);
			m_tasksSearch[it->second->m_id] = it->second;
			m_delaying.erase(it);
		}
	}
	task->signalCompleted();
}

/*! \brief Return the next task that should be run and mark it as executing
*/
std::shared_ptr<PooledTask> ThreadPool::getNextTask()
{
	Lock safety(&m_safe);
	if (!m_suspended && !m_tasks.empty())
	{
		std::shared_ptr<PooledTask> tmp = m_tasks.front();
		m_tasks.pop_front();
		m_tasksSearch.erase(tmp->m_id);
		m_executing[tmp->m_id] = tmp;
		return tmp;
	}

	return std::shared_ptr<PooledTask>();
}

/*! \brief Called by PooledThread to notify the ThreadPool that its running task has to wait for something
	\details This function can be called when the task failed to run to completion. It will be rescheduled
	at the end of the queue and its exec function will be called again.
	To notify the threadpool, a task should return 'false' from its exec() function.
	\param task The task that is to be paused
	\note The task itself is responsible for maintaining its state between the exec calls
	\note After calling this function the PooledThread should consider the supplied \a task as invalid since it's possible that it has been picked up, executed and deleted by another thread during the function return
*/
void ThreadPool::reportTaskPaused(std::shared_ptr<PooledTask> task)
{
	Lock safety(&m_safe);

	// remove task from 'executing' list
	TaskSet::iterator it = m_executing.find(task->m_id);
	if (it != m_executing.end())
		m_executing.erase(it);

	// add task back into either the delaying list if it should wait for something
	unsigned int waitForId = task->m_task->needToWaitFor();
	if (waitForId)
	{
		std::shared_ptr<PooledTask> after = findTask(waitForId);
		if (after)
		{
			after->m_dependentTasks.push_back(task);
			m_delaying[task->m_id] = task;
			return;
		}
	}

	// add task into ready queue
	m_tasks.push_back(task);
	m_tasksSearch[task->m_id] = task;
}

/*! \brief Suspend execution of tasks, any currently executing tasks will run to completion,
	but queued tasks will not be started.
	\param wait When set to true the function waits for all threads to finish their current task
	\sa resume
*/
void ThreadPool::suspend(bool wait) noexcept
{
	Lock safety(&m_safe);
	m_suspended = true;

	if (wait)
	{
		safety.unlock();

		for (ThreadSet::const_iterator it = m_threads.begin(); it != m_threads.end(); ++it)
			while ((*it)->isBusy())
				xsYield();
	}
}

/*! \brief Resume execution of tasks.
	\sa suspend
*/
void ThreadPool::resume()
{
	Lock safety(&m_safe);
	m_suspended = false;
}

/*! \brief Return the number of tasks executed (including paused) by the given thread
*/
unsigned int ThreadPool::executedCount(unsigned int thread) const
{
	unsigned int i = 0;
	for (ThreadSet::const_iterator it = m_threads.begin(); it != m_threads.end(); ++it, ++i)
		if (i == thread)
			return (*it)->executedCount();
	return 0;
}

/*! \brief Return the number of tasks completed by the given thread
*/
unsigned int ThreadPool::completedCount(unsigned int thread) const
{
	unsigned int i = 0;
	for (ThreadSet::const_iterator it = m_threads.begin(); it != m_threads.end(); ++it, ++i)
		if (i == thread)
			return (*it)->completedCount();
	return 0;
}

/*! \brief Return the number of tasks that failed to execute in the given thread
*/
unsigned int ThreadPool::failedCount(unsigned int thread) const
{
	unsigned int i = 0;
	for (ThreadSet::const_iterator it = m_threads.begin(); it != m_threads.end(); ++it, ++i)
		if (i == thread)
			return (*it)->failedCount();
	return 0;
}

ThreadPool* gPool = NULL;
bool gManagePool = true;

/*! \brief Return the global thread pool object, it will be created if it did not yet exist */
ThreadPool* ThreadPool::instance()
{
	if (gPool)
		return gPool;
	gPool = new ThreadPool;
	gManagePool = true;
	return gPool;
}

/*! \brief Destroy the global thread pool object
	\details If the pool was not created by this library, the current library will only stop using
	the pool, but will leave the object intact.
	The next call to instance() will create a new pool.
	\sa instance
*/
void ThreadPool::destroy()
{
	if (gPool && gManagePool)
		delete gPool;
	gPool = NULL;
	gManagePool = true;
}

/*! \brief Set the threadpool to use
	\details This function allows the user to supply his own ThreadPool object to be used
	by all subsequent operations. If another pool was created by this library, it will be
	destroyed.
	\param pool The new threadpool object to use. If NULL is supplied, it is simply destroyed
	or stopped from being used.
	\sa destroy
*/
void ThreadPool::setPool(ThreadPool* pool)
{
	if (pool != gPool)
	{
		destroy();
		if (pool)
		{
			gPool = pool;
			gManagePool = false;
		}
	}
}

/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////

/*! \class TaskCompletionWaiter
	\brief A class that allows multiple-dependency waiting
	\details A normal task in the ThreadPool can only wait for one task ID. This class allows waiting
	for multiple IDs. The IDs to wait for can be supplied upon creation or added later, but they
	should NOT be added once the object has been scheduled.
	The TaskCompletionWaiter must be scheduled in the ThreadPool that contains the task IDs to wait
	for so it can do its job.

	\note If you want a thread outside the ThreadPool to wait for multiple tasks to complete, create a
	TaskCompletionWaiter, schedule it and wait for the TaskCompletionWaiter to complete through
	the ThreadPool::waitForCompletion function.
*/

/*! \brief Constructor, sets up an empty waiter */
TaskCompletionWaiter::TaskCompletionWaiter(ThreadPool* pool)
	: ThreadPoolTask()
	, m_pool(pool)
{}

/*! \brief Constructor, sets up a waiter that waits for all items in the list to be completed */
TaskCompletionWaiter::TaskCompletionWaiter(const std::list<unsigned int>& waitlist, ThreadPool* pool)
	: ThreadPoolTask()
	, m_pool(pool)
	, m_waitList(waitlist)
{}

/*! \brief Destructor */
TaskCompletionWaiter::~TaskCompletionWaiter()
{}

/*! \brief task function, checks if there are tasks left that we should be waiting for
	\details The function returns false if at least one of the wait tasks still exists
	\returns True if successfully executed
*/
bool TaskCompletionWaiter::exec()
{
	while (m_waitList.size())
	{
		// if the task exists, reschedule this task to recheck after the task completes
		if (m_pool->doesTaskExist(*m_waitList.begin()))
			return false;

		m_waitList.erase(m_waitList.begin());
	}
	return true;
}

/*! \brief If there are wait tasks left, this function will return the id of the first in the list
	\returns The id of the first task in the list
*/
unsigned int TaskCompletionWaiter::needToWaitFor()
{
	if (m_waitList.size())
		return *m_waitList.begin();
	return 0;
}

/*! \brief Add the id of a task to wiat for to the list.
	\param id The ID to add
	\note This function should NOT be called once the task has been scheduled!
*/
void TaskCompletionWaiter::addWaitId(unsigned int id)
{
	m_waitList.push_back(id);
}
} // namespace xsens
