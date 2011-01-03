/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
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
		/** A thread-safe template queue for object passing between threads, with objects being passed being "T*".
		  * \sa mrpt::utils::CMessageQueue
		  */
		template <class T>
		class CThreadSafeQueue
		{
		protected:
			std::queue<T*> m_msgs; //!< The queue of messages. Memory is freed at destructor or by clients gathering messages.
			mrpt::synch::CCriticalSection			m_csQueue; //!< The critical section
		public:
			CThreadSafeQueue()
			{
			}
			
			virtual ~CThreadSafeQueue()
			{
				clear();
			}

			void clear() //!< Clear the queue of messages, freeing memory as required.
			{
				mrpt::synch::CCriticalSectionLocker locker( &m_csQueue );
				while (!m_msgs.empty())
				{
					delete m_msgs.front();
					m_msgs.pop();
				}		
			}

			void push( T *msg )  //!< Insert a new message in the queue - The object must be created with "new", and do not delete is after calling this, it must be deleted later.
			{
				mrpt::synch::CCriticalSectionLocker locker( &m_csQueue );
				m_msgs.push( msg );
			}

			/** Retrieve the next message in the queue, or NULL if there is no message.
			  *  The user MUST call "delete" with the returned object after use.
			  */
			T *get( )
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

			bool empty() const  //!< Return true if there are no messages.
			{
				mrpt::synch::CCriticalSectionLocker locker( &m_csQueue );
				return m_msgs.empty();		
			}

		}; // End of class def.

	} // End of namespace
} // end of namespace
#endif
