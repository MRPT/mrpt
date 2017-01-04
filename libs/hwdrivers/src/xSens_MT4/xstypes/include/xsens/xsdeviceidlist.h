/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef XSDEVICEIDLIST_H
#define XSDEVICEIDLIST_H

#include "xsdeviceidarray.h"

#define XsDeviceIdList XsDeviceIdArray

#ifndef __cplusplus
// obsolete:
#define XSDEVICEIDLIST_INITIALIZER		XSDEVICEIDARRAY_INITIALIZER
#define XsDeviceIdList_construct(thisPtr, sz, src)	XsDeviceIdArray_construct(thisPtr, sz, src)
#define XsDeviceIdList_assign(thisPtr, sz, src)		XsArray_assign(thisPtr, sz, src)
#define XsDeviceIdList_destruct(thisPtr)			XsArray_destruct(thisPtr)
#define XsDeviceIdList_copy(thisPtr, copy)			XsArray_copy(copy, thisPtr)
#define XsDeviceIdList_append(thisPtr, other)		XsArray_append(thisPtr, other)
#define XsDeviceIdList_popFront(thisPtr, count)		XsArray_erase(thisPtr, 0, count)
#define XsDeviceIdList_popBack(thisPtr, count)		XsArray_erase(thisPtr, (XsSize)-1, count)
#define XsDeviceIdList_swap(a, b)					XsArray_swap(a, b)
#define XsDeviceIdList_erase(thisPtr, index, count)	XsArray_erase(thisPtr, index, count)

#endif
#endif // file guard
