/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef XSOUTPUTCONFIGURATIONLIST_H
#define XSOUTPUTCONFIGURATIONLIST_H

#include "xsoutputconfigurationarray.h"
#define XsOutputConfigurationList	XsOutputConfigurationArray

#ifndef __cplusplus
// obsolete:
#define XSOUTPUTCONFIGURATIONLIST_INITIALIZER	XSOUTPUTCONFIGURATIONARRAY_INITIALIZER
#define XsOutputConfigurationList_assign(thisPtr, size, src)	XsArray_assign(thisPtr, size, src)
#define XsOutputConfigurationList_construct(thisPtr, size, src)	XsOutputConfigurationArray_construct(thisPtr, size, src)
#define XsOutputConfigurationList_destruct(thisPtr)				XsArray_destruct(thisPtr)
#define XsOutputConfigurationList_copy(thisPtr, copy)			XsArray_copy(copy, thisPtr)
#define XsOutputConfigurationList_append(thisPtr, other)		XsArray_append(thisPtr, other)
#define XsOutputConfigurationList_popFront(thisPtr, count)		XsArray_erase(thisPtr, 0, count)
#define XsOutputConfigurationList_popBack(thisPtr, count)		XsArray_erase(thisPtr, (XsSize)-1, count)
#define XsOutputConfigurationList_swap(a, b)					XsArray_swap(a, b)
#define XsOutputConfigurationList_erase(thisPtr, index, count)	XsArray_erase(thisPtr, index, count)
#define XsOutputConfigurationList_equal(a, b)					XsArray_equal(a, b)
#endif

#endif // file guard
