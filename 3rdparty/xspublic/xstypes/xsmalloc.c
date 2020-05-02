
//  Copyright (c) 2003-2019 Xsens Technologies B.V. or subsidiaries worldwide.
//  All rights reserved.
//  
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//  
//  1.	Redistributions of source code must retain the above copyright notice,
//  	this list of conditions, and the following disclaimer.
//  
//  2.	Redistributions in binary form must reproduce the above copyright notice,
//  	this list of conditions, and the following disclaimer in the documentation
//  	and/or other materials provided with the distribution.
//  
//  3.	Neither the names of the copyright holders nor the names of their contributors
//  	may be used to endorse or promote products derived from this software without
//  	specific prior written permission.
//  
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
//  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
//  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
//  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
//  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS 
//  SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES 
//  OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE 
//  ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.
//  

/*! \file
	This file contains platform-specific memory allocation routines
*/

#include "xsmalloc.h"
#if !(defined __ICCARM__) && !(defined _ADI_COMPILER) && !defined(__APPLE__) && !defined(__CRCC__) && !(defined(__arm__) && defined(__ARMCC_VERSION))
#include <malloc.h>
#endif
#include <stdlib.h>

#ifdef XSENS_ASSERT_MALLOC
#include <assert.h>
#undef XSENS_ASSERT_MALLOC
#define XSENS_ASSERT_MALLOC(x) assert(x)
#else
#define XSENS_ASSERT_MALLOC(x)
#endif

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

#ifndef _MSC_VER
#	ifdef __ANDROID__
#		define _aligned_malloc(size, align) memalign(align, size)
#	elif (defined __ICCARM__) || (defined _ADI_COMPILER) || (defined __CRCC__) || (defined IAR_ARM_CM3) || (defined __ARMEL__) || (defined(__arm__) && defined(__ARMCC_VERSION))
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

#endif //!_MSC_VER


//! \brief Allocates \a sz bytes of memory, optionally tracking the allocation
void* xsMalloc(size_t sz)
{
#ifdef TRACK_ALLOCS
	void* ptr = malloc(sz);
	XSENS_ASSERT_MALLOC(ptr);
	lastAllocIdx = (lastAllocIdx + 1) & (TRACK_ALLOCS-1);
	lastAllocs[lastAllocIdx] = ptr;
	return ptr;
#else
	void* ptr = malloc(sz);
	XSENS_ASSERT_MALLOC(ptr);
	return ptr;
#endif
}

//! \brief Reallocates \a sz bytes of memory, optionally tracking the allocation
void* xsRealloc(void* ptr, size_t sz)
{
#ifdef TRACK_ALLOCS
	lastFreeIdx = (lastFreeIdx + 1) & (TRACK_ALLOCS-1);
	lastFrees[lastFreeIdx] = ptr;

	ptr = realloc(ptr, sz);
	XSENS_ASSERT_MALLOC(ptr);
	lastAllocIdx = (lastAllocIdx + 1) & (TRACK_ALLOCS-1);
	lastAllocs[lastAllocIdx] = ptr;
	return ptr;
#else
	void* mem = realloc(ptr, sz);
	XSENS_ASSERT_MALLOC(mem);
	return mem;
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
	XSENS_ASSERT_MALLOC(ptr);
	lastAlignedAllocIdx = (lastAlignedAllocIdx + 1) & (TRACK_ALLOCS-1);
	lastAlignedAllocs[lastAlignedAllocIdx] = ptr;
	return ptr;
#else
	void* ptr = _aligned_malloc(sz, 16);
	XSENS_ASSERT_MALLOC(ptr);
	return ptr;
#endif
}

//! \brief Reallocates \a sz bytes of memory on a 16 byte boundary, optionally tracking the allocation
void* xsAlignedRealloc(void* ptr, size_t sz)
{
#ifdef TRACK_ALLOCS
	lastFreeIdx = (lastAlignedFreeIdx + 1) & (TRACK_ALLOCS-1);
	lastAlignedFrees[lastAlignedFreeIdx] = ptr;

	ptr = _aligned_realloc(ptr, sz, 16);
	XSENS_ASSERT_MALLOC(ptr);
	lastAlignedAllocIdx = (lastAlignedAllocIdx + 1) & (TRACK_ALLOCS-1);
	lastAlignedAllocs[lastAlignedAllocIdx] = ptr;
	return ptr;
#else
	void* mem = _aligned_realloc(ptr, sz, 16);
	XSENS_ASSERT_MALLOC(mem);
	return mem;
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



