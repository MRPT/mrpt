/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/core/alignment_req.h>

#include <memory>
#include <type_traits>

namespace mrpt
{
void* aligned_malloc(size_t size, size_t alignment);
void aligned_free(void* ptr);
/** Identical to aligned_malloc, but it zeroes the reserved memory block. */
void* aligned_calloc(size_t bytes, size_t alignment);

/** Aligned allocator that is compatible with C++11.
 * Default alignment can be 16 (default), 32 (if __AVX__ is defined) or 64
 * (if __AVX2__ is defined).
 * See: https://bitbucket.org/eigen/eigen/commits/f5b7700
 *
 * This was used (before May-2019) to provide custom STL aligned containers,
 * but the new(n,m) addition to C++17 rendered this needless (at last!)
 * See: http://eigen.tuxfamily.org/bz/show_bug.cgi?id=1409
 *
 * Anyway, this allocator class is left here just in case it is needed for
 * something else.
 */
template <class T, size_t AligmentBytes = MRPT_MAX_ALIGN_BYTES>
class aligned_allocator_cpp11 : public std::allocator<T>
{
   public:
	using size_type = std::size_t;
	using difference_type = std::ptrdiff_t;
	using pointer = T*;
	using const_pointer = const T*;
	using reference = T&;
	using const_reference = const T&;
	using value_type = T;

	template <class U>
	struct rebind
	{
		using other = aligned_allocator_cpp11<U>;
	};

	aligned_allocator_cpp11() : std::allocator<T>() {}
	aligned_allocator_cpp11(const aligned_allocator_cpp11& other)
		: std::allocator<T>(other)
	{
	}
	template <class U>
	aligned_allocator_cpp11(const aligned_allocator_cpp11<U>& other)
		: std::allocator<T>(other)
	{
	}
	~aligned_allocator_cpp11() = default;
	pointer allocate(size_type num, const void* /*hint*/ = nullptr)
	{
		return static_cast<pointer>(
			mrpt::aligned_malloc(num * sizeof(T), AligmentBytes));
	}
	void deallocate(pointer p, size_type /*num*/) { mrpt::aligned_free(p); }
};

}  // namespace mrpt
