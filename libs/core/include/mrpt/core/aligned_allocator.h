/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#pragma once

#include <type_traits>
#include <memory>
#include <mrpt/system/memory.h>

namespace mrpt
{
/** Aligned allocator that is compatible with C++11
 * See: https://bitbucket.org/eigen/eigen/commits/f5b7700
 */
template <class T, size_t AligmentBytes = 16>
class aligned_allocator_cpp11 : public std::allocator<T>
{
   public:
	typedef std::size_t size_type;
	typedef std::ptrdiff_t difference_type;
	typedef T* pointer;
	typedef const T* const_pointer;
	typedef T& reference;
	typedef const T& const_reference;
	typedef T value_type;

	template <class U>
	struct rebind
	{
		typedef aligned_allocator_cpp11<U> other;
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

	~aligned_allocator_cpp11() {}
	pointer allocate(size_type num, const void* /*hint*/ = nullptr)
	{
		return static_cast<pointer>(
			mrpt::system::os::aligned_malloc(num * sizeof(T), AligmentBytes));
	}

	void deallocate(pointer p, size_type /*num*/)
	{
		mrpt::system::os::aligned_free(p);
	}
};

/** Creates a `shared_ptr` with 16-byte aligned memory.
 * \ingroup mrpt_base_grp
 */
template <typename T, class... Args>
std::shared_ptr<T> make_aligned_shared(Args&&... args)
{
	typedef typename std::remove_const<T>::type T_nc;
	return std::allocate_shared<T>(
		mrpt::aligned_allocator_cpp11<T_nc>(), std::forward<Args>(args)...);
}
}
