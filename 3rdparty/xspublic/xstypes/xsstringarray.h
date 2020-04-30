
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
#endif
XSTYPES_DLL_API void XsStringArray_fromSplicedString(struct XsStringArray* thisPtr, struct XsString const* src, struct XsString const* separators);
XSTYPES_DLL_API void XsStringArray_join(struct XsStringArray const* thisPtr, struct XsString* result, struct XsString const* separator);

#ifdef __cplusplus
} // extern "C"

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
	inline explicit XsStringArray(XsString* ref, XsSize sz, XsDataFlags flags /* = XSDF_None */)
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

	/*! \brief Join the non-empty strings contained in the XsStringArray into one XsString, separating each item with \a separator
		\param separator An optional separator string to put between substrings
		\return The joined string
	*/
	XsString join(XsString const& separator) const
	{
		XsString tmp;
		XsStringArray_join(this, &tmp, &separator);
		return tmp;
	}

	/*! \copydoc XsStringArray_fromSplicedString */
	void fromSplicedString(XsString const& src, XsString const& separators)
	{
		XsStringArray_fromSplicedString(this, &src, &separators);
	}

	//! \brief Constructs an XsStringArray from an XsString \a src that is split up at each \a separator
	inline explicit XsStringArray(XsString const& src, XsString const& separators)
		: ArrayImpl()
	{
		XsStringArray_fromSplicedString(this, &src, &separators);
	}

	/*! \brief find same string in array which can be case insensitive or not
		\param needle String to be found
		\param isCaseSensitive Optional, default is true
		\return place of needle in the array or -1 if the string was not found
	*/
	int find(XsString const& needle, bool isCaseSensitive = true) const
	{
		if (isCaseSensitive)
			return XsArrayImpl<XsString, g_xsStringArrayDescriptor, XsStringArray>::find(needle);

		if (size() == 0)
			return -1;

		XsString needleLower(needle);
		for (XsString::iterator c = needleLower.begin(); c != needleLower.end(); ++c)
			*c = tolower(*c);

		for (XsSize i = 0; i < size(); ++i) // loop over all elements of the lists
		{
			XsString const& a = at(i);
			if (a.size() != needleLower.size())
				continue;

			bool found = true;
			for (XsSize j = 0; j < a.size(); ++j)
			{
				if (tolower(a[j]) != needleLower[j])
				{
					found = false;
					break;
				}
			}

			if (found)
				return (int)i;
		}
		return -1;
	}
};
#endif

#endif
