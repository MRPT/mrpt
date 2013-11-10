#ifndef XSPORTINFOLIST_H
#define XSPORTINFOLIST_H

#include "xsportinfoarray.h"

#define XsPortInfoList XsPortInfoArray

#ifndef __cplusplus
// obsolete:
#define XSPORTINFOLIST_INITIALIZER		XSPORTINFOARRAY_INITIALIZER
#define XsPortInfoList_assign(thisPtr, size, src)		XsArray_assign(thisPtr, sz, src)
#define XsPortInfoList_construct(thisPtr, size, src)	XsPortInfoArray_construct(thisPtr, size, src)
#define XsPortInfoList_destruct(thisPtr)				XsArray_destruct(thisPtr)
#define XsPortInfoList_copy(thisPtr, copy)				XsArray_copy(copy, thisPtr)
#define XsPortInfoList_append(thisPtr, other)			XsArray_append(thisPtr, other)
#define XsPortInfoList_erase(thisPtr, index, count)		XsArray_erase(thisPtr, index, count)
#define XsPortInfoList_swap(a, b)						XsArray_swap(a, b)
#endif

#endif // file guard
