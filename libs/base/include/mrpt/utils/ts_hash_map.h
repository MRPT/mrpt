/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
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
			unsigned int NUM_BYTES_HASH_TABLE = 2,
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

			struct const_iterator
			{
			public:
				const_iterator() : m_vec(NULL), m_idx_outer(0), m_idx_inner(0) {}
				const_iterator(const VECTOR_T &vec, size_t idx_outer, size_t idx_inner) : m_vec(const_cast<VECTOR_T*>(&vec)), m_idx_outer(idx_outer), m_idx_inner(idx_inner) {}
				const_iterator & operator = (const const_iterator& o) { m_vec = o.m_vec; m_idx_outer = o.m_idx_outer; m_idx_inner = o.m_idx_inner; return *this; }
				bool operator == (const const_iterator& o) { return m_vec == o.m_vec && m_idx_outer == o.m_idx_outer && m_idx_inner == o.m_idx_inner; }
				bool operator != (const const_iterator& o) { return !(*this==o); }
				const value_type & operator *() { return (*m_vec)[m_idx_outer][m_idx_inner]; }
				const value_type * operator ->() { return &(*m_vec)[m_idx_outer][m_idx_inner]; }
				inline const_iterator operator ++(int) { /* Post: it++ */ const_iterator aux = *this; ++(*this); return aux; }
				inline const_iterator& operator ++() {  /* pre: ++it */ incr(); return *this; }
			protected:
				VECTOR_T  *m_vec;
				size_t    m_idx_outer, m_idx_inner;
				void incr() {
					if (++m_idx_inner >= NUM_HAS_TABLE_COLLISIONS_ALLOWED) {
						m_idx_inner = 0;
						m_idx_outer++;
					}
				}
			};

			struct iterator : public const_iterator
			{
			public:
				iterator() : m_vec(NULL), m_idx_outer(0), m_idx_inner(0) {}
				iterator(VECTOR_T &vec, size_t idx_outer, size_t idx_inner) : m_vec(&vec), m_idx_outer(idx_outer), m_idx_inner(idx_inner) {}
				value_type & operator *() { return (*m_vec)[m_idx_outer][m_idx_inner]; }
				value_type * operator ->() { return &(*m_vec)[m_idx_outer][m_idx_inner]; }
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
			inline void clear() {
				throw std::runtime_error("todo");
			}

			/** Write/read via [i] operator, that creates all elements up to (and including) the i'th if they didn't exist already. */
			inline VALUE & operator[](const size_t i) {
				if (m_vec.size()<=i) m_vec.resize(i+1);
				m_vec[i].first=i;
				return m_vec[i].second;
			}

			bool empty() const {
				throw std::runtime_error("todo");
			}

			VALUE & operator [](const KEY &key) {
				throw std::runtime_error("todo");
			}
			const_iterator find(const KEY &key) const {
				throw std::runtime_error("todo");
			}

			iterator begin() { throw std::runtime_error("todo"); }
			iterator end() { throw std::runtime_error("todo"); }
			const_iterator begin() const { throw std::runtime_error("todo"); }
			const_iterator end() const { throw std::runtime_error("todo"); }

			/** @} */

		};  // end class ts_hash_map

	} // End of namespace
} // End of namespace
