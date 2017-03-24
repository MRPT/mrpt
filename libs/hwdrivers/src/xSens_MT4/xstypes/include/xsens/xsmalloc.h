/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef XSMALLOC_H
#define XSMALLOC_H

#include "xstypesconfig.h"
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

XSTYPES_DLL_API void* xsMalloc(size_t sz);
XSTYPES_DLL_API void* xsRealloc(void* ptr, size_t sz);
XSTYPES_DLL_API void  xsFree(void* ptr);

XSTYPES_DLL_API void* xsAlignedMalloc(size_t sz);
XSTYPES_DLL_API void* xsAlignedRealloc(void* ptr, size_t sz);
XSTYPES_DLL_API void  xsAlignedFree(void* ptr);

#ifndef xsMathMalloc
#define xsMathMalloc(n)		xsAlignedMalloc(n)
#endif

#ifndef xsMathRealloc
#define xsMathRealloc(p, n)	xsAlignedRealloc(p,n)
#endif

#ifndef xsMathFree
#define xsMathFree(p)		xsAlignedFree(p)
#endif

#ifdef __cplusplus
} // extern "C"
#endif

#endif // XSMALLOC_H
