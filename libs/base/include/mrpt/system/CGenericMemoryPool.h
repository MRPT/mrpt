/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | Copyright (c) 2012, University of Almeria                                 |
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
			bool                           & m_was_destroyed;  //!< With this trick we get rid of the "global destruction order fiasco" ;-)

			CGenericMemoryPool(const size_t max_pool_entries, bool &was_destroyed ) : m_maxPoolEntries(max_pool_entries), m_was_destroyed(was_destroyed)
			{
				m_was_destroyed = false;
			}

		public:
			inline size_t getMemoryPoolMaxSize() const                     { return m_maxPoolEntries; }
			inline void   setMemoryPoolMaxSize(const size_t maxNumEntries) { m_maxPoolEntries = maxNumEntries; }

			/** Construct-on-first-use (~singleton) pattern: Return the unique instance of this class for a given template arguments,
			  *  or NULL if it was once created but it's been destroyed (which means we're in the program global destruction phase).
			  */
			static CGenericMemoryPool<DATA_PARAMS,POOLABLE_DATA> * getInstance(const size_t max_pool_entries = 5)
			{
				static bool was_destroyed = false;
				static CGenericMemoryPool<DATA_PARAMS,POOLABLE_DATA> inst(max_pool_entries, was_destroyed);
				return was_destroyed ? NULL : &inst;
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
				{
					if (m_pool.begin()->second) delete m_pool.begin()->second;
					m_pool.erase(m_pool.begin());
				}

				m_pool.push_back( typename TList::value_type(params,block) );
			}

			~CGenericMemoryPool()
			{
				m_was_destroyed = true;
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
