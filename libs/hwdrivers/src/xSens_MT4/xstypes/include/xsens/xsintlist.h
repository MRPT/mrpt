/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef XSINTLIST_H
#define XSINTLIST_H

#include "xsintarray.h"

#define XsIntList	XsIntArray

#ifndef __cplusplus
// obsolete:
#define XSINTLIST_INITIALIZER		XsIntArray_INITIALIZER
#define XsIntList_construct(thisPtr, sz, src)	XsIntArray_construct(thisPtr, sz, src)
#define XsIntList_assign(thisPtr, sz, src)		XsArray_assign(thisPtr, sz, src)
#define XsIntList_destruct(thisPtr)				XsArray_destruct(thisPtr)
#define XsIntList_copy(thisPtr, copy)			XsArray_copy(copy, thisPtr)
#define XsIntList_append(thisPtr, other)		XsArray_append(thisPtr, other)
#define XsIntList_popFront(thisPtr, count)		XsArray_erase(thisPtr, 0, count)
#define XsIntList_popBack(thisPtr, count)		XsArray_erase(thisPtr, (XsSize)-1, count)
#define XsIntList_swap(a, b)					XsArray_swap(a, b)
#define XsIntList_erase(thisPtr, index, count)	XsArray_erase(thisPtr, index, count)
#define XsIntList_find(thisPtr, needle)			XsArray_find(thisPtr, needle)

#endif
#endif // file guard
