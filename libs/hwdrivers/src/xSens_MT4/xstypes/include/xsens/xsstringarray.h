/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef XSSTRINGARRAY_H
#define XSSTRINGARRAY_H

#include "xsarray.h"

#ifdef __cplusplus
#include "xsstring.h"
extern "C" {
#endif

extern XsArrayDescriptor const XSTYPES_DLL_API g_xsStringArrayDescriptor;

#ifndef __cplusplus
#define XSSTRINGARRAY_INITIALIZER	XSARRAY_INITIALIZER(&g_xsStringArrayDescriptor)
struct XsString;

XSARRAY_STRUCT(XsStringArray, struct XsString);
typedef struct XsStringArray XsStringArray;

XSTYPES_DLL_API void XsStringArray_construct(XsStringArray* thisPtr, XsSize count, struct XsString const* src);
#define XsStringArray_destruct(thisPtr)		XsArray_destruct(thisPtr)
#else
} // extern "C"
#endif

#ifdef __cplusplus
struct XsStringArray : public XsArrayImpl<XsString, g_xsStringArrayDescriptor, XsStringArray> {
	//! \brief Constructs an XsStringArray
	inline explicit XsStringArray(XsSize sz = 0, XsString const* src = 0)
		 : ArrayImpl(sz, src)
	{
	}

	//! \brief Constructs an XsStringArray as a copy of \a other
	inline XsStringArray(XsStringArray const& other)
		 : ArrayImpl(other)
	{
	}

	//! \brief Constructs an XsStringArray that references the data supplied in \a ref
	inline explicit XsStringArray(XsString* ref, XsSize sz, XsDataFlags flags = XSDF_None)
		: ArrayImpl(ref, sz, flags)
	{
	}

#ifndef XSENS_NOITERATOR
	//! \brief Constructs an XsStringArray with the array bound by the supplied iterators \a beginIt and \a endIt
	template <typename Iterator>
	inline XsStringArray(Iterator beginIt, Iterator endIt)
		: ArrayImpl(beginIt, endIt)
	{
	}
#endif
};
#endif

#endif // file guard
