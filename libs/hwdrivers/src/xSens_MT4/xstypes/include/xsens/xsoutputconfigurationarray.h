/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef XSOUTPUTCONFIGURATIONARRAY_H
#define XSOUTPUTCONFIGURATIONARRAY_H

#include "xsarray.h"

#ifdef __cplusplus
#include "xsoutputconfiguration.h"
extern "C" {
#endif

extern XsArrayDescriptor const XSTYPES_DLL_API g_xsOutputConfigurationArrayDescriptor;

#ifndef __cplusplus
#define XSOUTPUTCONFIGURATIONARRAY_INITIALIZER	XSARRAY_INITIALIZER(&g_xsOutputConfigurationArrayDescriptor)
struct XsOutputConfiguration;

XSARRAY_STRUCT(XsOutputConfigurationArray, struct XsOutputConfiguration);
typedef struct XsOutputConfigurationArray XsOutputConfigurationArray;

XSTYPES_DLL_API void XsOutputConfigurationArray_construct(XsOutputConfigurationArray* thisPtr, XsSize count, struct XsOutputConfiguration const* src);
#else
} // extern "C"
#endif

#ifdef __cplusplus
struct XsOutputConfigurationArray : public XsArrayImpl<XsOutputConfiguration, g_xsOutputConfigurationArrayDescriptor, XsOutputConfigurationArray> {
	//! \brief Constructs an XsOutputConfigurationArray
	inline explicit XsOutputConfigurationArray(XsSize sz = 0, XsOutputConfiguration const* src = 0)
		 : ArrayImpl(sz, src)
	{
	}

	//! \brief Constructs an XsOutputConfigurationArray as a copy of \a other
	inline XsOutputConfigurationArray(XsOutputConfigurationArray const& other)
		 : ArrayImpl(other)
	{
	}

	//! \brief Constructs an XsOutputConfigurationArray that references the data supplied in \a ref
	inline explicit XsOutputConfigurationArray(XsOutputConfiguration* ref, XsSize sz, XsDataFlags flags = XSDF_None)
		: ArrayImpl(ref, sz, flags)
	{
	}

#ifndef XSENS_NOITERATOR
	//! \brief Constructs an XsOutputConfigurationArray with the array bound by the supplied iterators \a beginIt and \a endIt
	template <typename Iterator>
	inline XsOutputConfigurationArray(Iterator beginIt, Iterator endIt)
		: ArrayImpl(beginIt, endIt)
	{
	}
#endif
};
#endif

#endif // file guard
