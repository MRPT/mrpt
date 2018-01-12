/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "core-precomp.h"  // Precompiled headers

#include <mrpt/core/aligned_allocator.h>

#include <cstdlib>  // free, posix_memalign, realloc, C++17 aligned_alloc
#if defined(_MSC_VER)
#include <malloc.h>  // _aligned_malloc()
#endif

void* mrpt::aligned_malloc(size_t size, size_t alignment)
{
#if !defined(_MSC_VER)
	// size must be an integral multiple of alignment:
	if ((size % alignment) != 0) size = ((size / alignment) + 1) * alignment;
	return ::aligned_alloc(alignment, size);
#else
	return _aligned_malloc(size, alignment);
#endif
}
void mrpt::aligned_free(void* ptr)
{
#if !defined(_MSC_VER)
	return ::free(ptr);
#else
	return _aligned_free(ptr);
#endif
}
void* mrpt::aligned_realloc(void* ptr, size_t size, size_t alignment)
{
#if !defined(_MSC_VER)
	// size must be an integral multiple of alignment:
	if ((size % alignment) != 0) size = ((size / alignment) + 1) * alignment;
	return std::realloc(ptr, size);
#else
	return _aligned_realloc(ptr, size, alignment);
#endif
}
