/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef XSSYNCSETTINGLIST_H
#define XSSYNCSETTINGLIST_H

#include "xssyncsettingarray.h"
#define XsSyncSettingList XsSyncSettingArray

#ifndef __cplusplus

#define XSSYNCSETTINGLIST_INITIALIZER		XSSYNCSETTINGSARRAY_INITIALIZER

#define XsSyncSettingList_assign(thisPtr, size, src)	XsArray_assign(thisPtr, size, src)
#define XsSyncSettingList_construct(thisPtr, size, src)	XsSyncSettingArray_construct(thisPtr, size, src)
#define XsSyncSettingList_destruct(thisPtr)				XsArray_destruct(thisPtr)
#define XsSyncSettingList_copy(thisPtr, copy)			XsArray_copy(copy, thisPtr)
#define XsSyncSettingList_append(thisPtr, other)		XsArray_append(thisPtr, other)
#define XsSyncSettingList_popFront(thisPtr, count)		XsArray_erase(thisPtr, 0, count)
#define XsSyncSettingList_popBack(thisPtr, count)		XsArray_erase(thisPtr, (XsSize)-1, count)
#define XsSyncSettingList_swap(a, b)					XsArray_swap(a, b)
#define XsSyncSettingList_erase(thisPtr, index, count)	XsArray_erase(thisPtr, index, count)

#endif
#endif // file guard
