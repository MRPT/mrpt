/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "xsmalloc.h"
#if !(defined __ICCARM__) && !(defined _ADI_COMPILER) && !defined(__APPLE__) && !defined(__CRCC__)
#include <malloc.h>
#endif
#ifndef _POSIX_C_SOURCE
#	define _POSIX_C_SOURCE 200809L
#endif
#include <stdlib.h>

#if !(defined __ICCARM__) && !(defined _ADI_COMPILER) && defined(XSENS_DEBUG)
//#define TRACK_ALLOCS	32
#endif

#ifdef TRACK_ALLOCS
int lastAllocIdx = -1;
void* lastAllocs[TRACK_ALLOCS];
int lastFreeIdx = -1;
void* lastFrees[TRACK_ALLOCS];

int lastAlignedAllocIdx = -1;
void* lastAlignedAllocs[TRACK_ALLOCS];
int lastAlignedFreeIdx = -1;
void* lastAlignedFrees[TRACK_ALLOCS];
#endif

#include <mrpt/config.h>
#ifndef HAVE_ALIGNED_MALLOC
#	ifdef __ANDROID__
#		define _aligned_malloc(size, align) memalign(align, size)
#	elif (defined __ICCARM__) || (defined _ADI_COMPILER) || (defined __CRCC__) || (defined IAR_ARM_CM3) || (defined __ARMEL__)
#		define _aligned_malloc(a, b) malloc(a)
#	else

void* __cdecl _aligned_malloc(size_t _Size, size_t _Alignment)
{
	void* rv = 0;
	int err = posix_memalign(&rv, _Alignment, _Size);
	if (err == 0)
		return rv;
	return NULL;
}

#	endif

#define _aligned_realloc(p, n, a)	realloc(p, n)
#define _aligned_free(_Memory)		free(_Memory)

#endif //!HAVE_ALIGNED_MALLOC  was: !_MSC_VER


//! \brief Allocates \a sz bytes of memory, optionally tracking the allocation
void* xsMalloc(size_t sz)
{
#ifdef TRACK_ALLOCS
	void* ptr = malloc(sz);
	lastAllocIdx = (lastAllocIdx + 1) & (TRACK_ALLOCS-1);
	lastAllocs[lastAllocIdx] = ptr;
	return ptr;
#else
	return malloc(sz);
#endif
}

//! \brief Reallocates \a sz bytes of memory, optionally tracking the allocation
void* xsRealloc(void* ptr, size_t sz)
{
#ifdef TRACK_ALLOCS
	lastFreeIdx = (lastFreeIdx + 1) & (TRACK_ALLOCS-1);
	lastFrees[lastFreeIdx] = ptr;

	ptr = realloc(ptr, sz);
	lastAllocIdx = (lastAllocIdx + 1) & (TRACK_ALLOCS-1);
	lastAllocs[lastAllocIdx] = ptr;
	return ptr;
#else
	return realloc(ptr, sz);
#endif
}

//! \brief Frees the memory pointed to by \a ptr, optionally tracking the allocation
void  xsFree(void* ptr)
{
#ifdef TRACK_ALLOCS
	lastFreeIdx = (lastFreeIdx + 1) & (TRACK_ALLOCS-1);
	lastFrees[lastFreeIdx] = ptr;
#endif
	free(ptr);
}

//! \brief Allocates \a sz bytes of memory on a 16 byte boundary, optionally tracking the allocation
void* xsAlignedMalloc(size_t sz)
{
#ifdef TRACK_ALLOCS
	void* ptr = _aligned_malloc(sz, 16);
	lastAlignedAllocIdx = (lastAlignedAllocIdx + 1) & (TRACK_ALLOCS-1);
	lastAlignedAllocs[lastAlignedAllocIdx] = ptr;
	return ptr;
#else
	return _aligned_malloc(sz, 16);
#endif
}

//! \brief Reallocates \a sz bytes of memory on a 16 byte boundary, optionally tracking the allocation
void* xsAlignedRealloc(void* ptr, size_t sz)
{
#ifdef TRACK_ALLOCS
	lastFreeIdx = (lastAlignedFreeIdx + 1) & (TRACK_ALLOCS-1);
	lastAlignedFrees[lastAlignedFreeIdx] = ptr;

	ptr = _aligned_realloc(ptr, sz, 16);
	lastAlignedAllocIdx = (lastAlignedAllocIdx + 1) & (TRACK_ALLOCS-1);
	lastAlignedAllocs[lastAlignedAllocIdx] = ptr;
	return ptr;
#else
	return _aligned_realloc(ptr, sz, 16);
#endif
}

//! \brief Frees the (aligned) memory pointed to by \a ptr, optionally tracking the allocation
void  xsAlignedFree(void* ptr)
{
#ifdef TRACK_ALLOCS
	lastAlignedFreeIdx = (lastAlignedFreeIdx + 1) & (TRACK_ALLOCS-1);
	lastAlignedFrees[lastAlignedFreeIdx] = ptr;
#endif
	_aligned_free(ptr);
}



