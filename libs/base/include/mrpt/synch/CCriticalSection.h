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
#ifndef  mrpt_synch_criticalsection_H
#define  mrpt_synch_criticalsection_H

#include <mrpt/utils/utils_defs.h>
#include <mrpt/utils/CStream.h>
#include <mrpt/utils/CReferencedMemBlock.h>

/*---------------------------------------------------------------
	Class
  ---------------------------------------------------------------*/
namespace mrpt
{
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
			utils::CReferencedMemBlock		m_data;   //!< The OS-dependent descriptors

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
