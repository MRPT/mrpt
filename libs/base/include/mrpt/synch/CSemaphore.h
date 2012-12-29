/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                                 |
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
