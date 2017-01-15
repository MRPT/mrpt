/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef XSINT64ARRAY_H
#define XSINT64ARRAY_H

#include "xsarray.h"
#include "pstdint.h"

#ifdef __cplusplus
extern "C" {
#endif

extern XsArrayDescriptor const XSTYPES_DLL_API g_xsInt64ArrayDescriptor;

#ifndef __cplusplus
#define XSINT64ARRAY_INITIALIZER	XSARRAY_INITIALIZER(&g_xsInt64ArrayDescriptor)
XSARRAY_STRUCT(XsInt64Array, int64_t);
typedef struct XsInt64Array XsInt64Array;

XSTYPES_DLL_API void XsInt64Array_construct(XsInt64Array* thisPtr, XsSize count, int64_t const* src);

#else
} // extern "C"
#endif

#ifdef __cplusplus
struct XsInt64Array : public XsArrayImpl<int64_t, g_xsInt64ArrayDescriptor, XsInt64Array> {
	//! \brief Constructs an XsInt64Array
	inline explicit XsInt64Array(XsSize sz = 0, int64_t const* src = 0)
		 : ArrayImpl(sz, src)
	{
	}

	//! \brief Constructs an XsInt64Array as a copy of \a other
	inline XsInt64Array(XsInt64Array const& other)
		 : ArrayImpl(other)
	{
	}

	//! \brief Constructs an XsInt64Array that references the data supplied in \a ref
	inline explicit XsInt64Array(int64_t* ref, XsSize sz, XsDataFlags flags = XSDF_None)
		: ArrayImpl(ref, sz, flags)
	{
	}

#ifndef XSENS_NOITERATOR
	//! \brief Constructs an XsInt64Array with the array bound by the supplied iterators \a beginIt and \a endIt
	template <typename Iterator>
	inline XsInt64Array(Iterator beginIt, Iterator endIt)
		: ArrayImpl(beginIt, endIt)
	{
	}
#endif
};
#endif


#endif // file guard
