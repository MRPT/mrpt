/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "core-precomp.h"  // Precompiled headers
#include <mrpt/core/aligned_allocator.h>
#include <cstdlib>  // free, realloc, C++17 aligned_alloc
#include <cstring>  // memset

void* mrpt::aligned_calloc(size_t bytes, size_t alignment)
{
	void* ptr = mrpt::aligned_malloc(bytes, alignment);
	if (ptr) ::memset(ptr, 0, bytes);
	return ptr;
}
void* mrpt::aligned_malloc(size_t size, size_t alignment)
{
	// size must be an integral multiple of alignment:
	if ((size % alignment) != 0) size = ((size / alignment) + 1) * alignment;
#ifdef _MSC_VER
	return _aligned_malloc(size, alignment);
#else
	return ::aligned_alloc(alignment, size);
#endif
}
void mrpt::aligned_free(void* ptr)
{
#ifdef _MSC_VER
	return _aligned_free(ptr);
#else
	return ::free(ptr);
#endif
}
void* mrpt::aligned_realloc(void* ptr, size_t size, size_t alignment)
{
	// size must be an integral multiple of alignment:
	if ((size % alignment) != 0) size = ((size / alignment) + 1) * alignment;
#ifdef _MSC_VER
	return _aligned_realloc(ptr, size, alignment);
#else
	return std::realloc(ptr, size);
#endif
}
