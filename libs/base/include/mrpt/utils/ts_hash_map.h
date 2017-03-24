/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/utils/compiler_fixes.h>
#include <mrpt/utils/mrpt_macros.h>
#include <mrpt/utils/integer_select.h>
#include <mrpt/utils/CArray.h>

namespace mrpt
{
	namespace utils
	{
		template <typename KEY,typename VALUE >
		struct ts_map_entry {
			bool  used;
			KEY   first;
			VALUE second;
			ts_map_entry() : used(false), first( KEY() ), second() {}
		};

		void BASE_IMPEXP reduced_hash(const std::string &value, uint8_t &hash); //!< hash function used by ts_hash_map. Uses dbj2 method
		void BASE_IMPEXP reduced_hash(const std::string &value, uint16_t &hash); //!< hash function used by ts_hash_map. Uses dbj2 method
		void BASE_IMPEXP reduced_hash(const std::string &value, uint32_t &hash); //!< hash function used by ts_hash_map. Uses dbj2 method
		void BASE_IMPEXP reduced_hash(const std::string &value, uint64_t &hash); //!< hash function used by ts_hash_map. Uses dbj2 method

		/** A thread-safe (ts) container which minimally emulates a std::map<>'s [] and find() methods but which is implemented as a linear vector indexed by a hash of KEY.
		  * Any custom hash function can be implemented, we don't rely by default on C++11 std::hash<> due to its limitation in some implementations.
		  *
		  * This implementation is much more efficient than std::map<> when the most common operation is accesing elements
		  *  by KEY with find() or [], and is also thread-safe if different threads create entries with different hash values.
		  *
		  * The default underlying non-associative container is a "memory-aligned std::vector<>", but it can be changed to a
		  *  standard vector<> or to a deque<> (to avoid memory reallocations) by changing the template parameter \a VECTOR_T.
		  *
		  * \note Defined in #include <mrpt/utils/ts_hash_map.h>
		  * \ingroup stlext_grp
		  */
		template <
			typename KEY,
			typename VALUE,
			unsigned int NUM_BYTES_HASH_TABLE = 1,
			unsigned int NUM_HAS_TABLE_COLLISIONS_ALLOWED = 5,
			typename VECTOR_T = mrpt::utils::CArray< mrpt::utils::CArray<ts_map_entry<KEY,VALUE>, NUM_HAS_TABLE_COLLISIONS_ALLOWED>, 1u << (8*NUM_BYTES_HASH_TABLE)>
			>
		class ts_hash_map
		{
		public:
			/** @name Types
			    @{ */
			typedef ts_hash_map<KEY, VALUE, NUM_BYTES_HASH_TABLE, NUM_HAS_TABLE_COLLISIONS_ALLOWED, VECTOR_T> self_t;
			typedef KEY                                     key_type;
			typedef ts_map_entry<KEY,VALUE>                 value_type;
			typedef VECTOR_T                                vec_t;

			struct iterator;
			struct const_iterator
			{
			public:
				const_iterator() : m_vec(NULL), m_parent(NULL), m_idx_outer(0), m_idx_inner(0) {}
				const_iterator(const VECTOR_T &vec, const self_t &parent, int idx_outer, int idx_inner) : m_vec(const_cast<VECTOR_T*>(&vec)), m_parent(const_cast<self_t*>(&parent)), m_idx_outer(idx_outer), m_idx_inner(idx_inner) {}
				const_iterator & operator = (const const_iterator& o) { m_vec = o.m_vec; m_idx_outer = o.m_idx_outer; m_idx_inner = o.m_idx_inner; return *this; }
				bool operator == (const const_iterator& o) const { return m_vec == o.m_vec && m_idx_outer == o.m_idx_outer && m_idx_inner == o.m_idx_inner; }
				bool operator != (const const_iterator& o) const { return !(*this==o); }
				const value_type & operator *() const { return (*m_vec)[m_idx_outer][m_idx_inner]; }
				const value_type * operator ->()  const { return &(*m_vec)[m_idx_outer][m_idx_inner]; }
				inline const_iterator operator ++(int) { /* Post: it++ */ const_iterator aux = *this; ++(*this); return aux; }
				inline const_iterator& operator ++() {  /* pre: ++it */ incr(); return *this; }
			protected:
				VECTOR_T  *m_vec;
				self_t    *m_parent;
				int        m_idx_outer, m_idx_inner;
				void incr() {
					// This loop ends with the first used entry in the nested arrays, or an iterator pointing to "end()".
					do {
						if (++m_idx_inner >= (int)NUM_HAS_TABLE_COLLISIONS_ALLOWED) {
							m_idx_inner = 0;
							m_idx_outer++;
						}
					} while (m_idx_outer<(int)m_parent->m_vec.size() && !(*m_vec)[m_idx_outer][m_idx_inner].used);
				}
			};

			struct iterator : public const_iterator
			{
			public:
				iterator() : const_iterator() {}
				iterator(VECTOR_T &vec, self_t &parent, int idx_outer, int idx_inner) : const_iterator(vec,parent,idx_outer,idx_inner) {}
				value_type & operator *() { return (*const_iterator::m_vec)[const_iterator::m_idx_outer][const_iterator::m_idx_inner]; }
				value_type * operator ->() { return &(*const_iterator::m_vec)[const_iterator::m_idx_outer][const_iterator::m_idx_inner]; }
				inline iterator operator ++(int) { /* Post: it++ */ iterator aux = *this; ++(*this); return aux; }
				inline iterator& operator ++() {  /* pre: ++it */ const_iterator::incr(); return *this; }
			};
			/** @} */
		private:
			vec_t  m_vec;  //!< The actual container
			size_t m_size; //!< Number of elements accessed with write access so far

		public:
			/** @name Constructors, read/write access and other operations
			    @{ */
			//!< Default constructor */
			ts_hash_map() : m_size(0)
			{ 
			}
			/** Clear the contents of this container */
			void clear() {
				m_size = 0;
				for (size_t oi = 0; oi < m_vec.size(); oi++)
					for (size_t ii = 0; ii < NUM_HAS_TABLE_COLLISIONS_ALLOWED; ii++)
						m_vec[oi][ii] = value_type();
			}

			bool empty() const { return m_size == 0; }

			/** Write/read via [i] operator, that creates an element if it didn't exist already. */
			VALUE & operator [](const KEY &key) {
				typename mrpt::utils::uint_select_by_bytecount<NUM_BYTES_HASH_TABLE>::type hash;
				reduced_hash(key, hash);
				mrpt::utils::CArray<ts_map_entry<KEY, VALUE>,NUM_HAS_TABLE_COLLISIONS_ALLOWED> & match_arr = m_vec[hash];
				for (unsigned int i = 0; i < NUM_HAS_TABLE_COLLISIONS_ALLOWED; i++)
				{
					if (!match_arr[i].used) {
						m_size++;
						match_arr[i].used = true;
						match_arr[i].first = key;
						return match_arr[i].second;
					}
					if (match_arr[i].first == key) return match_arr[i].second;
				}
				THROW_EXCEPTION("ts_hash_map: too many hash collisions!");
			}
			const_iterator find(const KEY &key) const {
				typename mrpt::utils::uint_select_by_bytecount<NUM_BYTES_HASH_TABLE>::type hash;
				reduced_hash(key, hash);
				const mrpt::utils::CArray<ts_map_entry<KEY, VALUE>, NUM_HAS_TABLE_COLLISIONS_ALLOWED> & match_arr = m_vec[hash];
				for (unsigned int i = 0; i < NUM_HAS_TABLE_COLLISIONS_ALLOWED; i++)
				{
					if (match_arr[i].used && match_arr[i].first == key)
						return const_iterator(m_vec,*this, hash,i);
				}
				return this->end();
			}

			const_iterator begin() const { const_iterator it(m_vec, *this, 0, -1); ++it; return it; }
			const_iterator end() const { return const_iterator(m_vec, *this, m_vec.size(), 0); }
			iterator begin() { iterator it(m_vec, *this, 0, -1); ++it; return it; }
			iterator end() { return iterator(m_vec, *this, m_vec.size(), 0); }

			/** @} */

		};  // end class ts_hash_map

	} // End of namespace
} // End of namespace
