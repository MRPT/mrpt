/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
   +---------------------------------------------------------------------------+ */
#ifndef  CThreadSafeQueue_H
#define  CThreadSafeQueue_H

#include <mrpt/utils/CMessage.h>
#include <mrpt/synch/CCriticalSection.h>
#include <queue>

namespace mrpt
{
	namespace utils
	{
		/** A thread-safe template queue for object passing between threads; for a template argument of T, the objects being passed in the queue are "T*".
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
		  *  Note that only dynamically allocated objects can be inserted with \a push() and that freeing that memory
		  *   if responsibility of the receiver of this queue as it receives objects with \a get(). However, elements
		  *   still in the queue upon destruction will be deleted automatically.
		  *
		  * \sa mrpt::utils::CMessageQueue
		 * \ingroup mrpt_base_grp
		  */
		template <class T>
		class CThreadSafeQueue
		{
		protected:
			std::queue<T*> m_msgs; //!< The queue of messages. Memory is freed at destructor or by clients gathering messages.
			mrpt::synch::CCriticalSection			m_csQueue; //!< The critical section
		public:
			/** Default ctor. */
			CThreadSafeQueue() { }

			virtual ~CThreadSafeQueue()
			{
				clear();
			}

			/** Clear the queue of messages, freeing memory as required. */
			void clear()
			{
				mrpt::synch::CCriticalSectionLocker locker( &m_csQueue );
				while (!m_msgs.empty())
				{
					delete m_msgs.front();
					m_msgs.pop();
				}
			}

			/** Insert a new message in the queue - The object must be created with "new", and do not delete is after calling this, it must be deleted later.
			  */
			inline void push( T *msg )
			{
				mrpt::synch::CCriticalSectionLocker locker( &m_csQueue );
				m_msgs.push( msg );
			}

			/** Retrieve the next message in the queue, or NULL if there is no message.
			  *  The user MUST call "delete" with the returned object after use.
			  */
			inline T *get( )
			{
				mrpt::synch::CCriticalSectionLocker locker( &m_csQueue );
				if (m_msgs.empty())
					return NULL;
				else
				{
					T *ret = m_msgs.front();
					m_msgs.pop();
					return ret;
				}
			}

			/** Skip all old messages in the queue and directly return the last one (the most recent, at the bottom of the queue), or NULL if there is no message.
			  *  \note The memory of all skipped messages is freed with "delete".
			  *  \note The user MUST call "delete" with the returned object after use.
			  */
			inline T *get_lastest_purge_old( )
			{
				mrpt::synch::CCriticalSectionLocker locker( &m_csQueue );
				if (m_msgs.empty())
					return NULL;
				else
				{
					for (;;)
					{
						T *ret = m_msgs.front();
						m_msgs.pop();
						if (m_msgs.empty()) return ret;
						else delete ret;
					}
				}
			}

			/** Return true if there are no messages. */
			bool empty() const
			{
				mrpt::synch::CCriticalSectionLocker locker( &m_csQueue );
				return m_msgs.empty();
			}

			/** Return the number of queued messages. */
			size_t size() const
			{
				mrpt::synch::CCriticalSectionLocker locker( &m_csQueue );
				return m_msgs.size();
			}

		}; // End of class def.

	} // End of namespace
} // end of namespace
#endif
