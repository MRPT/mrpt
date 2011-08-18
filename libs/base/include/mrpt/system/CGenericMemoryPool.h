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
#ifndef  MRPT_MEMORY_POOL_H
#define  MRPT_MEMORY_POOL_H

#include <mrpt/synch/CCriticalSection.h>

namespace mrpt
{
	namespace system
	{
		/** A generic system for versatile memory pooling.
		  *   This class implements the singleton pattern so a unique instance exists for each combination of template parameters.
		  *   All methods are thread-safe.
		  *
		  *   Basic usage:
		  *     - When needed, call \a request_memory() to check the availability of memory in the pool.
		  *		- At your class destructor, donate the memory to the pool with \a dump_to_pool().
		  *
		  *   Notice that memory requests are checked against memory blocks in the pool via a user-defined function:
		  *
		  *    bool POOLABLE_DATA::isSuitable(const POOLABLE_DATA & req) const { ... }
		  *
		  *   For an example of how to handle a memory pool, see the class mrpt::slam::CObservation3DRangeScan
		  *
		  *  \tparam POOLABLE_DATA A struct with user-defined objects which actually contain the memory blocks (e.g. one or more std::vector).
		  *  \tparam DATA_PARAMS A struct with user information about each memory block (e.g. size of a std::vector)
		  * \ingroup mrpt_memory
		  */
		template <class DATA_PARAMS,class POOLABLE_DATA>
		class CGenericMemoryPool
		{
		private:
			typedef std::list<std::pair<DATA_PARAMS,POOLABLE_DATA*> > TList;
			TList                          m_pool;
			mrpt::synch::CCriticalSection  m_pool_cs;
			size_t                         m_maxPoolEntries;

			CGenericMemoryPool() : m_maxPoolEntries(5)
			{
			}

		public:
			inline size_t getMemoryPoolMaxSize() const                     { return m_maxPoolEntries; }
			inline void   setMemoryPoolMaxSize(const size_t maxNumEntries) { m_maxPoolEntries = maxNumEntries; }

			/** Singleton: Return the unique instance of this class for a given template arguments: */
			static CGenericMemoryPool<DATA_PARAMS,POOLABLE_DATA> & getInstance()
			{
				static CGenericMemoryPool<DATA_PARAMS,POOLABLE_DATA> inst;
				return inst;
			}

			/** Request a block of data which fulfils the size requirements stated in \a params.
			  *  Notice that the decision on the suitability of each pool'ed block is done by DATA_PARAMS::isSuitable().
			  *  \return The block of data, or NULL if none suitable was found in the pool.
			  *  \note It is a responsibility of the user to free with "delete" the "POOLABLE_DATA" object itself once the memory has been extracted from its elements.
			  */
			POOLABLE_DATA * request_memory(const DATA_PARAMS &params)
			{
				// A quick check first:
				if (m_pool.empty()) return NULL;

				mrpt::synch::CCriticalSectionLocker lock( &m_pool_cs );
				for (typename TList::iterator it=m_pool.begin();it!=m_pool.end();++it) {
					if (it->first.isSuitable(params))
					{
						POOLABLE_DATA * ret = it->second;
						m_pool.erase(it);
						return ret;
					}
				}
				return NULL;
			}

			/** Saves the passed data block (characterized by \a params) to the pool.
			  *  If the overall size of the pool is above the limit, the oldest entry is removed.
			  *  \note It is a responsibility of the user to allocate in dynamic memory the "POOLABLE_DATA" object with "new".
			  */
			void dump_to_pool(const DATA_PARAMS &params, POOLABLE_DATA *block)
			{
				mrpt::synch::CCriticalSectionLocker lock( &m_pool_cs );

				while (m_pool.size()>=m_maxPoolEntries) // Free old data if needed
					m_pool.erase(m_pool.begin());

				m_pool.push_back( typename TList::value_type(params,block) );
			}

			~CGenericMemoryPool()
			{
				// Free remaining memory blocks:
				mrpt::synch::CCriticalSectionLocker lock( &m_pool_cs );
				for (typename TList::iterator it=m_pool.begin();it!=m_pool.end();++it)
					delete it->second;
				m_pool.clear();
			}
		};

	} // End of namespace
} // End of namespace

#endif
