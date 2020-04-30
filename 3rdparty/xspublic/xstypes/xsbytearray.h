
//  Copyright (c) 2003-2019 Xsens Technologies B.V. or subsidiaries worldwide.
//  All rights reserved.
//  
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//  
//  1.	Redistributions of source code must retain the above copyright notice,
//  	this list of conditions, and the following disclaimer.
//  
//  2.	Redistributions in binary form must reproduce the above copyright notice,
//  	this list of conditions, and the following disclaimer in the documentation
//  	and/or other materials provided with the distribution.
//  
//  3.	Neither the names of the copyright holders nor the names of their contributors
//  	may be used to endorse or promote products derived from this software without
//  	specific prior written permission.
//  
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
//  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
//  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
//  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
//  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS 
//  SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES 
//  OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE 
//  ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.
//  

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
#endif

#ifdef __cplusplus
} // extern "C"

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
	inline explicit XsByteArray(uint8_t* ref, XsSize sz, XsDataFlags flags /* = XSDF_None */)
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

	/*! \brief Return the data at position \a offset converted into a T
		\details This function will translate a part of the contained data into a type T. T needs to be a real
		POD type or structure with a trivial default constructor, since its contents will be completely
		overwritten by a memcpy
		\param offset The offset of the first byte to convert (in byte units, not in T units)
		\returns The converted value
	*/
	template <typename T>
	inline T getValue(XsSize offset) const
	{
		assert(offset+sizeof(T) <= size());
		T tmp;
		memcpy(&tmp, data()+offset, sizeof(T));
		return tmp;
	}

	/*! \brief Append \a value T to the byte array
		\param value The value to append to the array. The type T needs to be a POD structure or a primitive type
		since it gets copied into the array using memcpy.
	*/
	template <typename T>
	inline void appendValue(T const& value)
	{
		XsSize offset = size();
		resize(offset + sizeof(T));
		memcpy(data()+offset, &value, sizeof(T));
	}

	/*! \brief Set \a value T in the byte array at byte position \a offset
		\param value The value to append to the array. The type T needs to be a POD structure or a primitive type
		since it gets copied into the array using memcpy.
		\param offset The position of the first byte to write. The array will be enlarged if necessary
	*/
	template <typename T>
	inline void setValue(T const& value, XsSize offset)
	{
		if (size() < offset + sizeof(T))
			resize(offset + sizeof(T));
		memcpy(data()+offset, &value, sizeof(T));
	}
};
#endif
#endif
