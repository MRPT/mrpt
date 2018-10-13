/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mutex>
#include <queue>

namespace mrpt::containers
{
/** A thread-safe template queue for object passing between threads; for a
 * template argument of T, the objects being passed in the queue are "T*".
 *
 *  Usage example:
 *
 * \code
 * // Declaration:
 * CThreadSafeQueue<MyMsgType>  tsq;
 * ...
 *
 * // Thread 1: Write
 * {
 *   MyMsgType *msg = new MyMsgType;
 *   msg->...
 *   tsq.push(msg);  // Insert in the queue
 * }
 *
 * // Thread 2: Read
 * {
 *   MyMsgType *msg = tsq.get();
 *   if (msg)
 *   {
 *      // Process "msg"...
 *      delete msg;
 *   }
 * }
 * \endcode
 *
 *  Note that only dynamically allocated objects can be inserted with \a push()
 * and that freeing that memory
 *   if responsibility of the receiver of this queue as it receives objects
 * with \a get(). However, elements
 *   still in the queue upon destruction will be deleted automatically.
 *
 * \ingroup mrpt_containers_grp
 */
template <class T>
class CThreadSafeQueue
{
   protected:
	/** The queue of messages. Memory is freed at destructor or by clients
	 * gathering messages. */
	std::queue<T*> m_msgs;
	/** The critical section */
	mutable std::mutex m_csQueue;

   public:
	/** Default ctor. */
	CThreadSafeQueue() = default;
	virtual ~CThreadSafeQueue() { clear(); }
	/** Clear the queue of messages, freeing memory as required. */
	void clear()
	{
		std::lock_guard<std::mutex> lock(m_csQueue);
		while (!m_msgs.empty())
		{
			delete m_msgs.front();
			m_msgs.pop();
		}
	}

	/** Insert a new message in the queue - The object must be created with
	 * "new", and do not delete is after calling this, it must be deleted later.
	 */
	inline void push(T* msg)
	{
		std::lock_guard<std::mutex> lock(m_csQueue);
		m_msgs.push(msg);
	}

	/** Retrieve the next message in the queue, or nullptr if there is no
	 * message.
	 *  The user MUST call "delete" with the returned object after use.
	 */
	inline T* get()
	{
		std::lock_guard<std::mutex> lock(m_csQueue);
		if (m_msgs.empty())
			return nullptr;
		else
		{
			T* ret = m_msgs.front();
			m_msgs.pop();
			return ret;
		}
	}

	/** Skip all old messages in the queue and directly return the last one (the
	 * most recent, at the bottom of the queue), or nullptr if there is no
	 * message.
	 *  \note The memory of all skipped messages is freed with "delete".
	 *  \note The user MUST call "delete" with the returned object after use.
	 */
	inline T* get_lastest_purge_old()
	{
		std::lock_guard<std::mutex> lock(m_csQueue);
		if (m_msgs.empty())
			return nullptr;
		else
		{
			for (;;)
			{
				T* ret = m_msgs.front();
				m_msgs.pop();
				if (m_msgs.empty())
					return ret;
				else
					delete ret;
			}
		}
	}

	/** Return true if there are no messages. */
	bool empty() const
	{
		std::lock_guard<std::mutex> lock(m_csQueue);
		return m_msgs.empty();
	}

	/** Return the number of queued messages. */
	size_t size() const
	{
		std::lock_guard<std::mutex> lock(m_csQueue);
		return m_msgs.size();
	}

};  // End of class def.
}  // namespace mrpt::containers
