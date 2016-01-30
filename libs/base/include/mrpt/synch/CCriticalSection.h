/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  mrpt_synch_criticalsection_H
#define  mrpt_synch_criticalsection_H

#include <mrpt/utils/CReferencedMemBlock.h>
#include <string>

namespace mrpt
{
	namespace utils { class CStream; }

	/** @defgroup synch_grp Synchronization, multi-threading synch tools
	  * \ingroup mrpt_base_grp */


	/** This namespace provides multitask, synchronization utilities. \ingroup synch_grp
	 */
	namespace synch
	{
		/** This class provides simple critical sections functionality.
		  * \sa CCriticalSectionLocker
		  * \ingroup synch_grp
		  */
		class BASE_IMPEXP CCriticalSection
		{
		private:
			mrpt::utils::CReferencedMemBlock m_data;   //!< The OS-dependent descriptors

			std::string		m_name;
		public:
			/** Constructor
			  */
			CCriticalSection(const char *name = NULL);

			/** Destructor
			  */
			~CCriticalSection();

			/** Enter.
			  * \exception If the calling thread already possesses this critical section (it would be a dead-lock).
			  */
			void  enter() const;

			/** Leave
			  * \exception If the calling thread is not the current owener of the critical section.
			  */
			void  leave() const;

			/** Returns the name used in the constructor. */
			std::string getName() const { return m_name; }

			/** If set to a non-NULL value, debug messages regarding the calling threads IDs will be output.
			  */
			utils::CStream		*m_debugOut;
		};

		/** A class acquiring a CCriticalSection at its constructor, and releasing it at destructor.
		  *   It is a better idea to always use CCriticalSectionLocker, since it is more secure in the case of possible exceptions, many different exit points from a function, etc.. : it will always release the critical section at the destructor.
		  *    Example:
		  *  \code
		  *		{  // Code in this scope is protected by critical section
		  *			CCriticalSectionLocker  myCSLocker( &myCS );
		  *			...
		  *		}  // End of code protected by critical section
		  *  \endcode
		  *  \sa CCriticalSection, THREADSAFE_OPERATION
		  */
		class BASE_IMPEXP CCriticalSectionLocker
		{
		protected:
			const CCriticalSection	*m_cs;

		public:
			/** Constructor: enters the critical section.
			  * \note [Since MRPT 0.9.6] The pointer can be NULL, in which case no action at all will be taken.
			  */
			CCriticalSectionLocker( const CCriticalSection *cs  );

			CCriticalSectionLocker(const CCriticalSectionLocker &o) : m_cs(o.m_cs)
			{
			}

			CCriticalSectionLocker & operator = (const CCriticalSectionLocker&o)
			{
				m_cs = o.m_cs;
				return *this;
			}

			/** Destructor: leaves the critical section.
			  */
			~CCriticalSectionLocker();

		}; // end of CCriticalSectionLocker



		/** A macro for protecting a given piece of code with a critical section; for example:
		  *  \code
		  *    CCriticalSection  cs;
		  *    MyObject  obj;
		  *    ...
		  *
		  *    THREADSAFE_OPERATION(cs,  obj.foo(); )
		  *    ...
		  *    THREADSAFE_OPERATION(cs,  obj.foo(); obj.bar(); }
		  *
		  *  \endcode
		  *
		  * \sa CCriticalSectionLocker, CThreadSafeVariable
		  */
		#define  THREADSAFE_OPERATION(_CRITSECT_OBJ, CODE_TO_EXECUTE )  \
				{ \
					mrpt::synch::CCriticalSectionLocker lock(&_CRITSECT_OBJ); \
					CODE_TO_EXECUTE \
				}


	} // End of namespace
} // End of namespace

#endif
