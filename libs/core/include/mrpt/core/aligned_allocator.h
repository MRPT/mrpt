/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#pragma once

#include <type_traits>
#include <memory>

namespace mrpt
{
void* aligned_malloc(size_t size, size_t alignment);
void* aligned_realloc(void* ptr, size_t size, size_t alignment);
void aligned_free(void* ptr);

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
			mrpt::aligned_malloc(num * sizeof(T), AligmentBytes));
	}

	void deallocate(pointer p, size_type /*num*/) { mrpt::aligned_free(p); }
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

/** Put this macro inside any class with members that require 16-byte memory
 * alignment (e.g. Eigen matrices), to override default new()/delete().
 * Obviously, this macro is only *required* if the class is to be instantiated
 * in dynamic memory.
 */
#define MRPT_MAKE_ALIGNED_OPERATOR_NEW                                         \
	void* operator new(size_t size) { return mrpt::aligned_malloc(size, 16); } \
	void* operator new[](size_t size)                                          \
	{                                                                          \
		return mrpt::aligned_malloc(size, 16);                                 \
	}                                                                          \
	void operator delete(void* ptr) noexcept { mrpt::aligned_free(ptr); }      \
	void operator delete[](void* ptr) noexcept { mrpt::aligned_free(ptr); }    \
	/* in-place new and delete. since (at least afaik) there is no actual   */ \
	/* memory allocated we can safely let the default implementation handle */ \
	/* this particular case. */                                                \
	static void* operator new(size_t size, void* ptr)                          \
	{                                                                          \
		return ::operator new(size, ptr);                                      \
	}                                                                          \
	void operator delete(void* memory, void* ptr) noexcept                     \
	{                                                                          \
		return ::operator delete(memory, ptr);                                 \
	}                                                                          \
	/* nothrow-new (returns zero instead of std::bad_alloc) */                 \
	void* operator new(size_t size, const std::nothrow_t&) noexcept            \
	{                                                                          \
		try                                                                    \
		{                                                                      \
			return mrpt::aligned_malloc(size, 16);                             \
		}                                                                      \
		catch (...)                                                            \
		{                                                                      \
			return nullptr;                                                    \
		}                                                                      \
	}                                                                          \
	void operator delete(void* ptr, const std::nothrow_t&)noexcept             \
	{                                                                          \
		mrpt::aligned_free(ptr);                                               \
	}
}
