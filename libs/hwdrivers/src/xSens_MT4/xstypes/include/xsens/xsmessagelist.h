/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef XSMESSAGELIST_H
#define XSMESSAGELIST_H

#include "xsmessagearray.h"
#define XsMessageList XsMessageArray

#ifndef __cplusplus
// obsolete:
#define XSMESSAGELIST_INITIALIZER XSMESSAGEARRAY_INITIALIZER
#define XsMessageList_assign(thisPtr, size, src)	XsArray_assign(thisPtr, size, src)
#define XsMessageList_destruct(thisPtr)				XsArray_destruct(thisPtr)
#define XsMessageList_copy(thisPtr, copy)			XsArray_copy(copy, thisPtr)
#define XsMessageList_append(thisPtr, other)		XsArray_append(thisPtr, other)
#define XsMessageList_popFront(thisPtr, count)		XsArray_erase(thisPtr, 0, count)
#define XsMessageList_popBack(thisPtr, count)		XsArray_erase(thisPtr, (XsSize) -1, count)
#define XsMessageList_swap(a, b)					XsArray_swap(a, b)
#define XsMessageList_erase(thisPtr, index, count)	XsArray_erase(thisPtr, index, count)
#endif

#endif // file guard
