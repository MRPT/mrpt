/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef XSDEVICEIDARRAY_H
#define XSDEVICEIDARRAY_H

#include "xsarray.h"
#include "xsdeviceid.h"

#ifdef __cplusplus
extern "C" {
#endif

extern XsArrayDescriptor const XSTYPES_DLL_API g_xsDeviceIdArrayDescriptor;

#ifndef __cplusplus
#define XSDEVICEIDARRAY_INITIALIZER	XSARRAY_INITIALIZER(&g_xsDeviceIdArrayDescriptor)
XSARRAY_STRUCT(XsDeviceIdArray, XsDeviceId);
typedef struct XsDeviceIdArray XsDeviceIdArray;

XSTYPES_DLL_API void XsDeviceIdArray_construct(XsDeviceIdArray* thisPtr, XsSize count, XsDeviceId const* src);
#else
} // extern "C"
#endif

#ifdef __cplusplus
struct XsDeviceIdArray : public XsArrayImpl<XsDeviceId, g_xsDeviceIdArrayDescriptor, XsDeviceIdArray> {
	//! \brief Constructs an XsDeviceIdArray
	inline explicit XsDeviceIdArray(XsSize sz = 0, XsDeviceId const* src = 0)
		 : ArrayImpl(sz, src)
	{
	}

	//! \brief Constructs an XsDeviceIdArray as a copy of \a other
	inline XsDeviceIdArray(XsDeviceIdArray const& other)
		 : ArrayImpl(other)
	{
	}

	//! \brief Constructs an XsDeviceIdArray that references the data supplied in \a ref
	inline explicit XsDeviceIdArray(XsDeviceId* ref, XsSize sz, XsDataFlags flags = XSDF_None)
		: ArrayImpl(ref, sz, flags)
	{
	}
#ifndef XSENS_NOITERATOR
	//! \brief Constructs an XsDeviceIdArray with the array bound by the supplied iterators \a beginIt and \a endIt
	template <typename Iterator>
	inline XsDeviceIdArray(Iterator beginIt, Iterator endIt)
		: ArrayImpl(beginIt, endIt)
	{
	}
#endif
};
#endif


#endif // file guard
