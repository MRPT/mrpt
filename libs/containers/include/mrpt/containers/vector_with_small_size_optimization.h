/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/core/aligned_allocator.h>  // aligned_allocator_cpp11

#include <array>
#include <cstddef>	// size_t
#include <type_traits>	// conditional_t, ...
#include <vector>

namespace mrpt::containers
{
namespace internal
{
struct UnspecializedBool
{
	operator const bool&() const { return b; }
	operator bool&() { return b; }

	bool b;
};
}  // namespace internal

/** Container that transparently and dynamically switches between a std::array
 * and std::vector. Used to avoid heap allocations with small vectors.
 *
 * \note In `#include <mrpt/containers/vector_with_small_size_optimization.h>`
 * \ingroup mrpt_containers_grp
 */
template <typename VAL, size_t small_size, size_t alignment = 16>
class vector_with_small_size_optimization
{
   private:
	using T = std::conditional_t<
		std::is_same_v<VAL, bool>, internal::UnspecializedBool, VAL>;
	using ALLOC = mrpt::aligned_allocator_cpp11<T>;
	using self_t =
		vector_with_small_size_optimization<VAL, small_size, alignment>;
	using large_vec = std::vector<T, ALLOC>;
	using small_array = std::array<T, small_size>;

	/** @name Data
	 * @{ */
	large_vec m_v;
	alignas(alignment) small_array m_a;
	bool m_is_small = true;
	size_t m_size = 0;
	/** @} */

   public:
	using value_type = T;
	using reference = T&;
	using const_reference = const T&;
	using difference_type = typename large_vec::difference_type;
	using size_type = typename large_vec::size_type;

	vector_with_small_size_optimization() = default;
	~vector_with_small_size_optimization() = default;

	vector_with_small_size_optimization(size_t n)
		: m_is_small(n <= small_size), m_size(n)
	{
		if (!m_is_small) m_v.resize(n);
	}

	vector_with_small_size_optimization(
		const vector_with_small_size_optimization& o)
	{
		*this = o;
	}
	vector_with_small_size_optimization(vector_with_small_size_optimization&& o)
	{
		*this = o;
	}
	vector_with_small_size_optimization& operator=(
		const vector_with_small_size_optimization& o)
	{
		m_size = o.m_size;
		m_is_small = o.m_is_small;
		if (m_size > small_size) m_v = o.m_v;
		else if (m_size > 0)
			m_a = o.m_a;
		return *this;
	}
	vector_with_small_size_optimization& operator=(
		vector_with_small_size_optimization&& o)
	{
		m_size = o.m_size;
		m_is_small = o.m_is_small;
		if (m_size > small_size) m_v = std::move(o.m_v);
		else if (m_size > 0)
			m_a = std::move(o.m_a);
		return *this;
	}

	template <typename TYPE, typename POINTER, typename REFERENCE>
	class iteratorImpl
	{
		using STORAGE = std::conditional_t<
			std::is_same_v<POINTER, bool*>, internal::UnspecializedBool*,
			std::conditional_t<
				std::is_same_v<POINTER, const bool*>,
				const internal::UnspecializedBool*, POINTER>>;
		using self = iteratorImpl<TYPE, POINTER, REFERENCE>;

	   public:
		using value_type = TYPE;
		using reference = REFERENCE;
		using pointer = POINTER;
		using iterator_category = std::random_access_iterator_tag;
		using difference_type = typename large_vec::difference_type;
		iteratorImpl() = default;
		iteratorImpl(STORAGE ptr) : m_ptr(ptr) {}
		self operator++()
		{
			self i = *this;
			m_ptr++;
			return i;
		}
		self operator--()
		{
			self i = *this;
			m_ptr--;
			return i;
		}
		self operator++(int)
		{
			m_ptr++;
			return *this;
		}
		self operator--(int)
		{
			m_ptr--;
			return *this;
		}
		self operator+(difference_type n)
		{
			self i = *this;
			i.m_ptr += n;
			return i;
		}
		self operator-(difference_type n)
		{
			self i = *this;
			i.m_ptr -= n;
			return i;
		}
		self operator+=(difference_type n)
		{
			m_ptr += n;
			return *this;
		}
		self operator-=(difference_type n)
		{
			m_ptr -= n;
			return *this;
		}
		difference_type operator-(const self& o) const
		{
			return m_ptr - o.m_ptr;
		}
		REFERENCE operator*() { return *m_ptr; }
		const REFERENCE operator*() const { return *m_ptr; }
		POINTER operator->() { return m_ptr; }
		const POINTER operator->() const { return m_ptr; }
		bool operator==(const self& o) { return m_ptr == o.m_ptr; }
		bool operator!=(const self& o) { return m_ptr != o.m_ptr; }

	   private:
		STORAGE m_ptr{nullptr};
	};

	using iterator = iteratorImpl<VAL, VAL*, VAL&>;
	using const_iterator = iteratorImpl<VAL, const VAL*, const VAL&>;

	void resize(size_type n)
	{
		if (m_size)
		{
			if (m_is_small && n > small_size)
			{
				m_v.assign(m_a.begin(), m_a.begin() + m_size);
			}
			else if (!m_is_small && n <= small_size)
			{
				std::copy(m_v.begin(), m_v.begin() + n, m_a.begin());
			}
		}
		m_size = n;
		m_is_small = (n <= small_size);
		if (!m_is_small) { m_v.resize(m_size); }
	}

	void fill(const T& v)
	{
		if (m_is_small) m_a.fill(v);
		else
			m_v.assign(m_v.size(), v);
	}

	size_t size() const { return m_size; }
	bool empty() const { return m_size == 0; }

	reference operator[](size_type n) { return m_is_small ? m_a[n] : m_v[n]; }

	const_reference operator[](size_type n) const
	{
		return m_is_small ? m_a[n] : m_v[n];
	}

	/** Like [], but throws an exception if accessing out of bounds.
	 * \note (Note in MRPT 2.3.3)
	 */
	reference at(size_type n) { return m_is_small ? m_a.at(n) : m_v.at(n); }

	/** Like [], but throws an exception if accessing out of bounds.
	 * \note (Note in MRPT 2.3.3)
	 */
	const_reference at(size_type n) const
	{
		return m_is_small ? m_a.at(n) : m_v.at(n);
	}

	const_reference back() const
	{
		return m_is_small ? m_a[m_size - 1] : m_v.back();
	}
	reference back() { return m_is_small ? m_a[m_size - 1] : m_v.back(); }

	const_reference front() const
	{
		return m_is_small ? m_a.front() : m_v.front();
	}
	reference front() { return m_is_small ? m_a.front() : m_v.front(); }

	void swap(self_t& x)
	{
		if (m_is_small && x.m_is_small) { m_a.swap(x.m_a); }
		else if (!m_is_small && !x.m_is_small)
		{
			m_v.swap(x.m_v);
		}
		else if (!m_is_small && x.m_is_small)
		{
			std::copy(x.m_a.begin(), x.m_a.begin() + x.m_size, m_a.begin());
			x.m_v.swap(m_v);
		}
		else
		{
			m_v.swap(x.m_v);
			std::copy(m_a.begin(), m_a.begin() + m_size, x.m_a.begin());
		}
		std::swap(m_size, x.m_size);
		std::swap(m_is_small, x.m_is_small);
	}

	iterator begin() noexcept { return m_is_small ? m_a.data() : m_v.data(); }
	const_iterator begin() const noexcept
	{
		return m_is_small ? m_a.data() : m_v.data();
	}

	iterator end() noexcept
	{
		return m_is_small ? m_a.data() + m_size : m_v.data() + m_size;
	}
	const_iterator end() const noexcept
	{
		return m_is_small ? m_a.data() + m_size : m_v.data() + m_size;
	}

	/** Grows the container by one and writes the value in the new final
	 * position.
	 * \note (Note in MRPT 2.3.3)
	 */
	void push_back(const VAL& val)
	{
		const auto idx = size();
		resize(idx + 1);
		at(idx) = val;
	}
};

}  // namespace mrpt::containers
