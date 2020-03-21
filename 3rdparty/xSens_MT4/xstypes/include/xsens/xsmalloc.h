/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#ifndef XSMALLOC_H
#define XSMALLOC_H

#include <stddef.h>
#include "xstypesconfig.h"

#ifdef __cplusplus
extern "C"
{
#endif

	XSTYPES_DLL_API void* xsMalloc(size_t sz);
	XSTYPES_DLL_API void* xsRealloc(void* ptr, size_t sz);
	XSTYPES_DLL_API void xsFree(void* ptr);

	XSTYPES_DLL_API void* xsAlignedMalloc(size_t sz);
	XSTYPES_DLL_API void* xsAlignedRealloc(void* ptr, size_t sz);
	XSTYPES_DLL_API void xsAlignedFree(void* ptr);

#ifndef xsMathMalloc
#define xsMathMalloc(n) xsAlignedMalloc(n)
#endif

#ifndef xsMathRealloc
#define xsMathRealloc(p, n) xsAlignedRealloc(p, n)
#endif

#ifndef xsMathFree
#define xsMathFree(p) xsAlignedFree(p)
#endif

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // XSMALLOC_H
