/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
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
#ifndef  mrpt_synch_semaphore_H
#define  mrpt_synch_semaphore_H

#include <mrpt/utils/utils_defs.h>
#include <mrpt/utils/CStream.h>
#include <mrpt/utils/CReferencedMemBlock.h>

/*---------------------------------------------------------------
	Class
  ---------------------------------------------------------------*/
namespace mrpt
{
	namespace synch
	{
		/** A semaphore for inter-thread synchronization.
		  * The state of a semaphore object is signaled when its count is greater than zero,
		  *  and nonsignaled when its count is equal to zero. The initialCount parameter specifies
		  *  the initial count. Each time a waiting thread is released because of the semaphore's
		  *  signaled state, the count of the semaphore is decreased by one. Use the release function
		  *  to increment a semaphore's count by a specified amount. The count can never be less
		  *   than zero or greater than the value specified in the maxCount parameter.
		  * \ingroup synch_grp
		  */
		class BASE_IMPEXP CSemaphore
		{
		protected:
			utils::CReferencedMemBlock		m_data;
			std::string  m_name; //!< The name of the named semaphore, or empty if unnamed.

        public:
            /** Creates a semaphore.
              *  If \a name is not an empty string, a named semaphore is created. In that case
              *   if the semaphore didn't exist it's created. Otherwise, the existing semaphore
              *   is linked to this object and then \a initialCount and \a maxCount are ignored.
              */
            CSemaphore(
                unsigned int    initialCount,
                unsigned int    maxCount,
                const std::string &name=std::string("") );

            /** Destructor
              */
            virtual ~CSemaphore();

            /** Blocks until the count of the semaphore to be non-zero.
              * \param timeout_ms The timeout in milliseconds, or set to zero to wait indefinidely.
              * \return true if the semaphore has been signaled, false on timeout or any other error.
              */
            bool waitForSignal( unsigned int timeout_ms = 0 );

            /** Increments the count of the semaphore by a given amount.
              */
            void release(unsigned int increaseCount = 1);


			/** Get the name of the named semaphore or an empty string if it's unnamed */
			inline std::string  getName() const { return m_name; }

			/** Return true if this is a named semaphore */
			inline bool isNamed() const { return !m_name.empty(); }


		};

	} // End of namespace

} // End of namespace

#endif
