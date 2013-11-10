#ifndef XSMESSAGEARRAY_H
#define XSMESSAGEARRAY_H

#include "xsarray.h"
#include "pstdint.h"

#ifdef __cplusplus
#include "xsmessage.h"
extern "C" {
#endif

extern XsArrayDescriptor const XSTYPES_DLL_API g_xsMessageArrayDescriptor;

#ifndef __cplusplus
#define XSMESSAGEARRAY_INITIALIZER	XSARRAY_INITIALIZER(&g_xsMessageArrayDescriptor)

struct XsMessage;
XSARRAY_STRUCT(XsMessageArray, struct XsMessage);
typedef struct XsMessageArray XsMessageArray;

XSTYPES_DLL_API void XsMessageArray_construct(XsMessageArray* thisPtr, XsSize count, struct XsMessage const* src);
#else
} // extern "C"
#endif

#ifdef __cplusplus
struct XsMessageArray : public XsArrayImpl<XsMessage, g_xsMessageArrayDescriptor, XsMessageArray> {
	//! \brief Constructs an XsMessageArray
	inline XsMessageArray(XsSize sz = 0, XsMessage const* src = 0)
		 : ArrayImpl(sz, src)
	{
	}

	//! \brief Constructs an XsMessageArray as a copy of \a other
	inline XsMessageArray(XsMessageArray const& other)
		 : ArrayImpl(other)
	{
	}

	//! \brief Constructs an XsMessageArray that references the data supplied in \a ref
	inline explicit XsMessageArray(XsMessage* ref, XsSize sz, XsDataFlags flags = XSDF_None)
		: ArrayImpl(ref, sz, flags)
	{
	}

	//! \brief Constructs an XsMessageArray with the array bound by the supplied iterators \a beginIt and \a endIt
	template <typename Iterator>
	inline XsMessageArray(Iterator beginIt, Iterator endIt)
		: ArrayImpl(beginIt, endIt)
	{
	}
};
#endif

#endif // file guard
