/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef XSDEBUGCOUNTERS_H
#define XSDEBUGCOUNTERS_H

#include "xsatomicint.h"

// These counters are only available in the non-public version of the XsTypes library
// For the public version, they will all always have a 0 value.

#ifdef __cplusplus
extern "C" {
#endif

extern XsAtomicInt XSTYPES_DLL_API XsVector_allocCount;
extern XsAtomicInt XSTYPES_DLL_API XsVector_freeCount;

extern XsAtomicInt XSTYPES_DLL_API XsMatrix_allocCount;
extern XsAtomicInt XSTYPES_DLL_API XsMatrix_freeCount;

extern XsAtomicInt XSTYPES_DLL_API XsArray_allocCount;
extern XsAtomicInt XSTYPES_DLL_API XsArray_freeCount;

#ifdef __cplusplus
} // extern "C"
#endif

#endif
