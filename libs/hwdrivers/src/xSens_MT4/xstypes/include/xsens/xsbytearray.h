/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef XSBYTEARRAY_H
#define XSBYTEARRAY_H

#include "xsarray.h"
#include "pstdint.h"

#ifdef __cplusplus
#include "xsstring.h"
extern "C" {
#endif

extern XsArrayDescriptor const XSTYPES_DLL_API g_xsByteArrayDescriptor;

#ifndef __cplusplus
#define XSBYTEARRAY_INITIALIZER	XSARRAY_INITIALIZER(&g_xsByteArrayDescriptor)
XSARRAY_STRUCT(XsByteArray, uint8_t);
typedef struct XsByteArray XsByteArray;
XSTYPES_DLL_API void XsByteArray_construct(XsByteArray* thisPtr, XsSize count, uint8_t const* src);

// obsolete:
#define XsByteArray_ref(thisPtr, sz, src, flags)	XsArray_ref(thisPtr, sz, src, flags)
#define XsByteArray_assign(thisPtr, sz, src)		XsArray_assign(thisPtr, sz, src)
#define XsByteArray_destruct(thisPtr)				XsArray_destruct(thisPtr)
#define XsByteArray_copy(thisPtr, copy)				XsArray_copy(copy, thisPtr)
#define XsByteArray_append(thisPtr, other)			XsArray_append(thisPtr, other)
#define XsByteArray_popFront(thisPtr, count)		XsArray_erase(thisPtr, 0, count)
#define XsByteArray_popBack(thisPtr, count)			XsArray_erase(thisPtr, (XsSize)-1, count)
#define XsByteArray_fromString(str, copy)			XsArray_assign(copy, str->m_size?str->m_size:1, str->m_size?str->m_data:"\0")
#define XsByteArray_swap(a, b)						XsArray_swap(a, b)
#define XsByteArray_erase(thisPtr, index, count)	XsArray_erase(thisPtr, index, count)

#else
} // extern "C"
#endif

#ifdef __cplusplus
struct XsByteArray : public XsArrayImpl<uint8_t, g_xsByteArrayDescriptor, XsByteArray> {
	//! \brief Constructs an XsByteArray
	inline explicit XsByteArray(XsSize sz = 0, uint8_t const* src = 0)
		 : ArrayImpl(sz, src)
	{
	}

	//! \brief Constructs an XsByteArray as a copy of \a other
	inline XsByteArray(XsByteArray const& other)
		 : ArrayImpl(other)
	{
	}

	//! \brief Constructs an XsByteArray that references the data supplied in \a ref
	inline explicit XsByteArray(uint8_t* ref, XsSize sz, XsDataFlags flags = XSDF_None)
		: ArrayImpl(ref, sz, flags)
	{
	}
#ifndef XSENS_NOITERATOR
	//! \brief Constructs an XsByteArray with the array bound by the supplied iterators \a beginIt and \a endIt
	template <typename Iterator>
	inline XsByteArray(Iterator beginIt, Iterator endIt)
		: ArrayImpl(beginIt, endIt)
	{
	}
#endif
	//! \brief Constructs an XsByteArray as a copy of the supplied XsString, including the terminating 0
	inline XsByteArray(XsString const& src)
		: ArrayImpl()
	{
		assign(src.size()+1, reinterpret_cast<uint8_t const*>(src.c_str()));
	}

	//! \brief Return a pointer to the internal data buffer
	inline uint8_t* data() { return begin().operator ->(); }

	//! \brief Return a pointer to the internal data buffer
	inline uint8_t const* data() const { return begin().operator ->(); }
};
#endif
#endif // file guard
