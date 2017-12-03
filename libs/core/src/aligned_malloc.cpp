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

#include <cstdlib>  // for free, posix_memalign, realloc
#if defined(_MSC_VER)
#include <malloc.h> // _aligned_malloc()
#endif

void * mrpt::aligned_malloc(size_t size, size_t alignment)
{
#if defined(__GNUC__) && !defined(__MINGW32__)
	return ::aligned_malloc(size, alignment);
#else
	return _aligned_malloc(size, alignment);
#endif
}
void mrpt::aligned_free(void * ptr)
{
#if defined(__GNUC__) && !defined(__MINGW32__)
	return ::aligned_free(ptr);
#else
	return _aligned_free(ptr);
#endif
}
