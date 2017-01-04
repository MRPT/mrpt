/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  MRPT_MEMORY_H
#define  MRPT_MEMORY_H

#include <mrpt/utils/core_defs.h>
#include <cstring>

namespace mrpt
{
	namespace system
	{
		/** \addtogroup mrpt_memory Memory utilities (in #include <mrpt/system/memory.h>)
		  * \ingroup mrpt_base_grp
		  *  @{ */

		/** Returns the memory occupied by this process, in bytes */
		unsigned long  BASE_IMPEXP getMemoryUsage();

		/** In platforms and compilers with support to "alloca", allocate a memory block on the stack; if alloca is not supported, it is emulated as a normal "malloc" - NOTICE: Since in some platforms alloca will be emulated with malloc, alloca_free MUST BE ALWAYS CALLED to avoid memory leaks.
		  *   This method MUST BE a macro rather than a function in order to operate on the caller's stack.
		  *  \sa mrpt_alloca_free
		  */
#if defined(_MSC_VER) && (_MSC_VER>=1400)
		// Visual Studio 2005, 2008
#	define		mrpt_alloca( nBytes )	_malloca(nBytes)
#elif defined(HAVE_ALLOCA)
		// GCC
#	define		mrpt_alloca( nBytes )	::alloca(nBytes)
#else
	// Default: Emulate with memory in the heap:
#	define		mrpt_alloca( nBytes )	::malloc( nBytes )
#endif

		/** This method must be called to "free" each memory block allocated with "system::alloca": If the block was really allocated in the stack, no operation is actually performed, otherwise it will be freed from the heap.
		  *   This method MUST BE a macro rather than a function in order to operate on the caller's stack.
		  * \sa mrpt_alloca
		  */
#if defined(_MSC_VER) && (_MSC_VER>=1400)
		// Visual Studio 2005, 2008
#	define		mrpt_alloca_free( mem_block )	_freea(mem_block)
#elif defined(HAVE_ALLOCA)
		// GCC
#	define		mrpt_alloca_free( mem_block )
#else
	// Default: Emulate with memory in the heap:
#	define		mrpt_alloca_free( mem_block )	free(mem_block)
#endif

		/** @} */

		namespace os
		{
			/** \addtogroup mrpt_memory Memory utilities
			  *  @{ */

			/** Returns an aligned memory block.
			  * \param alignment The desired alignment, typ. 8 or 16 bytes. 1 means no alignment required. It must be a power of two.
			  * \sa aligned_free, aligned_realloc, aligned_calloc
			  * \note Based on code by William Chan
			*/
			void BASE_IMPEXP *aligned_malloc(size_t bytes, size_t alignment);

			/** Identical to aligned_malloc, but it zeroes the reserved memory block. */
			inline void *aligned_calloc(size_t bytes, size_t alignment)
			{
				void *ptr = mrpt::system::os::aligned_malloc(bytes, alignment);
				if (ptr) ::memset(ptr,0,bytes);
				return ptr;
			}

			/** Frees a memory block reserved by aligned_malloc.
			  * \param alignment The desired alignment, typ. 8 or 16 bytes. 1 means no alignment required.
			  * If old_ptr is NULL, a new block will be reserved from scratch.
			  * \sa aligned_malloc, aligned_free
			  */
			void BASE_IMPEXP  *aligned_realloc(void* old_ptr, size_t bytes, size_t alignment);

			/** Frees a memory block reserved by aligned_malloc
			  * \sa aligned_malloc
			  */
			void BASE_IMPEXP aligned_free(void* p);

			/** Returns a pointer a bit forward in memory so it's aligned for the given boundary size
			   * \note Function copied from OpenCV with a different name to avoid conflicts.
			   */
			template<typename _Tp> inline _Tp* align_ptr(_Tp* ptr, int n=(int)sizeof(_Tp))
			{
				return (_Tp*)(((size_t)ptr + n-1) & -n);
			}

			/** @} */
		}	// end namespace "os"

	/** \addtogroup mrpt_memory Memory utilities
	  *  @{ */
	// The following templates are taken from libcvd (LGPL). See http://mi.eng.cam.ac.uk/~er258/cvd/
	// Check if the pointer is aligned to the specified byte granularity
	template<int bytes> bool is_aligned(const void* ptr);
	template<> inline bool is_aligned<8>(const void* ptr) {   return ((reinterpret_cast<size_t>(ptr)) & 0x7) == 0;   }
	template<> inline bool is_aligned<16>(const void* ptr) {  return ((reinterpret_cast<size_t>(ptr)) & 0xF) == 0;   }
	/** @} */


	// A version of MRPT_MAKE_ALIGNED_OPERATOR_NEW that doesn't force including the entire Eigen lib:
	#define MRPT_MAKE_ALIGNED_OPERATOR_NEW \
		void *operator new(size_t size)  { return mrpt::system::os::aligned_malloc(size,16); } \
		void *operator new[](size_t size){ return mrpt::system::os::aligned_malloc(size,16); } \
		void operator delete(void * ptr) MRPT_NO_THROWS { mrpt::system::os::aligned_free(ptr); } \
		void operator delete[](void * ptr) MRPT_NO_THROWS { mrpt::system::os::aligned_free(ptr); } \
		/* in-place new and delete. since (at least afaik) there is no actual   */ \
		/* memory allocated we can safely let the default implementation handle */ \
		/* this particular case. */ \
		static void *operator new(size_t size, void *ptr) { return ::operator new(size,ptr); } \
		void operator delete(void * memory, void *ptr) MRPT_NO_THROWS { return ::operator delete(memory,ptr); } \
		/* nothrow-new (returns zero instead of std::bad_alloc) */ \
		void* operator new(size_t size, const std::nothrow_t&) MRPT_NO_THROWS { try { return mrpt::system::os::aligned_malloc(size,16); } catch (...) { return 0; } } \
		void operator delete(void *ptr, const std::nothrow_t&) MRPT_NO_THROWS { mrpt::system::os::aligned_free(ptr); }

	} // End of namespace
} // End of namespace

#endif
