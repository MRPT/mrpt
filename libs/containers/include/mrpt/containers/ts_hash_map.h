/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/core/common.h>  // remove MSVC warnings
#include <mrpt/core/integer_select.h>
#include <mrpt/core/lock_helper.h>

#include <array>
#include <mutex>
#include <stdexcept>
#include <string_view>

namespace mrpt::containers
{
template <typename KEY, typename VALUE>
struct ts_map_entry
{
	bool used{false};
	KEY first;
	VALUE second;

	ts_map_entry() = default;
	ts_map_entry(const ts_map_entry& e) { *this = e; }
	ts_map_entry& operator=(const ts_map_entry& e)
	{
		used = !!e.used;
		first = e.first;
		second = e.second;
		return *this;
	}
	ts_map_entry(ts_map_entry&& e) { *this = std::move(e); }
	ts_map_entry& operator=(ts_map_entry&& e)
	{
		used = e.used;
		first = e.first;
		second = e.second;
		return *this;
	}
};

/** hash function used by ts_hash_map. Uses dbj2 method */
void reduced_hash(const std::string_view& value, uint8_t& hash);
void reduced_hash(const std::string_view& value, uint16_t& hash);
void reduced_hash(const std::string_view& value, uint32_t& hash);
void reduced_hash(const std::string_view& value, uint64_t& hash);

/** A thread-safe (ts) container which minimally emulates a std::map<>'s [] and
 * find() methods but which is implemented as a linear vector indexed by a hash
 * of KEY.
 * Any custom hash function can be implemented, we don't rely by default on
 * C++11 std::hash<> due to its limitation in some implementations.
 *
 * This implementation is much more efficient than std::map<> when the most
 * common operation is accessing elements
 *  by KEY with find() or [], and is also thread-safe if different threads
 * create entries with different hash values.
 *
 * The default underlying non-associative container is a "memory-aligned
 * std::vector<>", but it can be changed to a
 *  standard vector<> or to a deque<> (to avoid memory reallocations) by
 * changing the template parameter \a VECTOR_T.
 *
 * \note Defined in #include <mrpt/containers/ts_hash_map.h>
 * \ingroup mrpt_containers_grp
 */
template <
	typename KEY, typename VALUE, unsigned int NUM_BYTES_HASH_TABLE = 1,
	unsigned int NUM_HAS_TABLE_COLLISIONS_ALLOWED = 5,
	typename VECTOR_T = std::array<
		std::array<ts_map_entry<KEY, VALUE>, NUM_HAS_TABLE_COLLISIONS_ALLOWED>,
		1u << (8 * NUM_BYTES_HASH_TABLE)>>
class ts_hash_map
{
   public:
	/** @name Types
		@{ */
	using self_t = ts_hash_map<
		KEY, VALUE, NUM_BYTES_HASH_TABLE, NUM_HAS_TABLE_COLLISIONS_ALLOWED,
		VECTOR_T>;
	using key_type = KEY;
	using value_type = ts_map_entry<KEY, VALUE>;
	using vec_t = VECTOR_T;

	struct iterator;
	struct const_iterator
	{
	   public:
		const_iterator() : m_vec(nullptr), m_parent(nullptr) {}
		const_iterator(
			const VECTOR_T& vec, const self_t& parent, int idx_outer,
			int idx_inner)
			: m_vec(const_cast<VECTOR_T*>(&vec)),
			  m_parent(const_cast<self_t*>(&parent)),
			  m_idx_outer(idx_outer),
			  m_idx_inner(idx_inner)
		{
		}
		const_iterator(const const_iterator& o) { *this = o; }
		const_iterator& operator=(const const_iterator& o)
		{
			m_vec = o.m_vec;
			m_idx_outer = o.m_idx_outer;
			m_idx_inner = o.m_idx_inner;
			return *this;
		}
		bool operator==(const const_iterator& o) const
		{
			return m_vec == o.m_vec && m_idx_outer == o.m_idx_outer &&
				m_idx_inner == o.m_idx_inner;
		}
		bool operator!=(const const_iterator& o) const { return !(*this == o); }
		const value_type& operator*() const
		{
			return (*m_vec)[m_idx_outer][m_idx_inner];
		}
		const value_type* operator->() const
		{
			return &(*m_vec)[m_idx_outer][m_idx_inner];
		}
		inline const_iterator operator++(int)
		{ /* Post: it++ */
			const_iterator aux = *this;
			++(*this);
			return aux;
		}
		inline const_iterator& operator++()
		{ /* pre: ++it */
			incr();
			return *this;
		}

	   protected:
		VECTOR_T* m_vec;
		self_t* m_parent;
		int m_idx_outer{0}, m_idx_inner{0};
		void incr()
		{
			// This loop ends with the first used entry in the nested arrays, or
			// an iterator pointing to "end()".
			do
			{
				if (++m_idx_inner >=
					static_cast<int>(NUM_HAS_TABLE_COLLISIONS_ALLOWED))
				{
					m_idx_inner = 0;
					m_idx_outer++;
				}
			} while (m_idx_outer < static_cast<int>(m_parent->m_vec.size()) &&
					 !(*m_vec)[m_idx_outer][m_idx_inner].used);
		}
	};

	struct iterator : public const_iterator
	{
	   public:
		iterator() : const_iterator() {}
		iterator(VECTOR_T& vec, self_t& parent, int idx_outer, int idx_inner)
			: const_iterator(vec, parent, idx_outer, idx_inner)
		{
		}
		value_type& operator*()
		{
			return (*const_iterator::m_vec)[const_iterator::m_idx_outer]
										   [const_iterator::m_idx_inner];
		}
		value_type* operator->()
		{
			return &(*const_iterator::m_vec)[const_iterator::m_idx_outer]
											[const_iterator::m_idx_inner];
		}
		inline iterator operator++(int)
		{ /* Post: it++ */
			iterator aux = *this;
			++(*this);
			return aux;
		}
		inline iterator& operator++()
		{ /* pre: ++it */
			const_iterator::incr();
			return *this;
		}
	};
	/** @} */
   private:
	/** The actual container */
	vec_t m_vec;
	/** Number of elements accessed with write access so far */
	size_t m_size{0};

	std::recursive_mutex m_mtx;	 //!< for m_vec and m_size

   public:
	/** @name Constructors, read/write access and other operations
		@{ */
	//!< Default constructor */
	ts_hash_map() = default;

	ts_hash_map(const ts_hash_map& o) { *this = o; }
	ts_hash_map(ts_hash_map&& o) { *this = std::move(o); }

	ts_hash_map& operator=(const ts_hash_map& o)
	{
		auto lck1 = mrpt::lockHelper(m_mtx);
		auto lck2 = mrpt::lockHelper(o.m_mtx);
		m_vec = o.m_vec;
		m_size = o.m_size;
		return *this;
	}
	ts_hash_map& operator=(ts_hash_map&& o)
	{
		auto lck1 = mrpt::lockHelper(m_mtx);
		auto lck2 = mrpt::lockHelper(o.m_mtx);
		m_vec = std::move(o.m_vec);
		m_size = o.m_size;
		return *this;
	}

	/** Clear the contents of this container */
	void clear()
	{
		auto lck = mrpt::lockHelper(m_mtx);
		m_size = 0;
		for (size_t oi = 0; oi < m_vec.size(); oi++)
			for (size_t ii = 0; ii < NUM_HAS_TABLE_COLLISIONS_ALLOWED; ii++)
				m_vec[oi][ii] = value_type();
	}

	bool empty() const
	{
		auto lck = mrpt::lockHelper(m_mtx);
		return m_size == 0;
	}

	/** noexcept version of operator[], returns nullptr upon failure */
	VALUE* find_or_alloc(const KEY& key) noexcept
	{
		auto lck = mrpt::lockHelper(m_mtx);

		mrpt::uint_select_by_bytecount_t<NUM_BYTES_HASH_TABLE> hash;
		reduced_hash(key, hash);
		auto& match_arr = m_vec[hash];
		for (unsigned int i = 0; i < NUM_HAS_TABLE_COLLISIONS_ALLOWED; i++)
		{
			if (!match_arr[i].used)
			{
				m_size++;
				match_arr[i].used = true;
				match_arr[i].first = key;
				return &match_arr[i].second;
			}
			if (match_arr[i].first == key) return &match_arr[i].second;
		}
		return nullptr;
	}

	/** Write/read via [i] operator, that creates an element if it didn't exist
	 * already. */
	VALUE& operator[](const KEY& key)
	{
		auto lck = mrpt::lockHelper(m_mtx);

		VALUE* v = find_or_alloc(key);
		if (!v)
			throw std::runtime_error("ts_hash_map: too many hash collisions!");
		return *v;
	}

	const_iterator find(const KEY& key) const
	{
		auto lck = mrpt::lockHelper(m_mtx);

		mrpt::uint_select_by_bytecount_t<NUM_BYTES_HASH_TABLE> hash;
		reduced_hash(key, hash);
		auto& match_arr = m_vec[hash];
		for (unsigned int i = 0; i < NUM_HAS_TABLE_COLLISIONS_ALLOWED; i++)
		{
			if (match_arr[i].used && match_arr[i].first == key)
				return const_iterator(m_vec, *this, hash, i);
		}
		return this->end();
	}

	const_iterator begin() const
	{
		auto lck = mrpt::lockHelper(m_mtx);

		const_iterator it(m_vec, *this, 0, -1);
		++it;
		return it;
	}
	const_iterator end() const
	{
		auto lck = mrpt::lockHelper(m_mtx);
		return const_iterator(m_vec, *this, m_vec.size(), 0);
	}
	iterator begin()
	{
		auto lck = mrpt::lockHelper(m_mtx);
		iterator it(m_vec, *this, 0, -1);
		++it;
		return it;
	}
	iterator end()
	{
		auto lck = mrpt::lockHelper(m_mtx);
		return iterator(m_vec, *this, m_vec.size(), 0);
	}
	/** @} */

};	// end class ts_hash_map

}  // namespace mrpt::containers
