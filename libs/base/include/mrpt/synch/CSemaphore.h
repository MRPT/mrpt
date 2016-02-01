/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  mrpt_synch_semaphore_H
#define  mrpt_synch_semaphore_H

#include <mrpt/utils/CReferencedMemBlock.h>
#include <string>

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
			mrpt::utils::CReferencedMemBlock m_data;
			std::string  m_name; //!< The name of the named semaphore, or empty if unnamed.

        public:
            /** Creates a semaphore.
              *  If \a name is not an empty string, a named semaphore is created. In that case
              *   if the semaphore didn't exist it's created. Otherwise, the existing semaphore
              *   is linked to this object and then \a initialCount and \a maxCount are ignored.
              *  \note Named semaphores require Linux kernel version>2.6.12
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
