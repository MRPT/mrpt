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

// This is to match Eigen expectations on alignment of dynamic objects:
#if defined(__AVX2__) || defined(EIGEN_VECTORIZE_AVX512)
#define MRPT_MAX_ALIGN_BYTES 64
#elif defined(__AVX__)
#define MRPT_MAX_ALIGN_BYTES 32
#else
#define MRPT_MAX_ALIGN_BYTES 16
#endif

namespace mrpt
{
void* aligned_malloc(size_t size, size_t alignment);
void* aligned_realloc(void* ptr, size_t size, size_t alignment);
void aligned_free(void* ptr);
/** Identical to aligned_malloc, but it zeroes the reserved memory block. */
inline void* aligned_calloc(size_t bytes, size_t alignment);

/** Aligned allocator that is compatible with C++11.
 * Default alignment can be 16 (default), 32 (if __AVX__ is defined) or 64
 * (if __AVX2__ is defined).
 * See also: https://bitbucket.org/eigen/eigen/commits/f5b7700
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

/** Creates a `shared_ptr` with aligned memory via aligned_allocator_cpp11<>.
 * \ingroup mrpt_base_grp
 */
template <typename T, class... Args>
std::shared_ptr<T> make_aligned_shared(Args&&... args)
{
	using T_nc = typename std::remove_const<T>::type;
	return std::allocate_shared<T>(
		mrpt::aligned_allocator_cpp11<T_nc>(), std::forward<Args>(args)...);
}

/** Put this macro inside any class with members that require {16,32,64}-byte
 * memory alignment (e.g. Eigen matrices), to override default new()/delete().
 * Obviously, this macro is only *required* if the class is to be instantiated
 * in dynamic memory.
 */
#define MRPT_MAKE_ALIGNED_OPERATOR_NEW                                         \
	void* operator new(size_t size)                                            \
	{                                                                          \
		return mrpt::aligned_malloc(size, MRPT_MAX_ALIGN_BYTES);               \
	}                                                                          \
	void* operator new[](size_t size)                                          \
	{                                                                          \
		return mrpt::aligned_malloc(size, MRPT_MAX_ALIGN_BYTES);               \
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
			return mrpt::aligned_malloc(size, MRPT_MAX_ALIGN_BYTES);           \
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
}  // namespace mrpt
