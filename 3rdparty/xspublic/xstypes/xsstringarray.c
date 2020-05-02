
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

#include "xsstringarray.h"
#include "xsstring.h"

/*! \struct XsStringArray
	\brief A list of XsString values
	\sa XsArray
*/

//! \brief Descriptor for XsStringArray
XsArrayDescriptor const g_xsStringArrayDescriptor = {
	sizeof(XsString),
	XSEXPCASTITEMSWAP XsArray_swap,
	XSEXPCASTITEMMAKE XsString_construct,
	XSEXPCASTITEMCOPY XsArray_copyConstruct,
	XSEXPCASTITEMMAKE XsArray_destruct,
	XSEXPCASTITEMCOPY XsArray_copy,
	XSEXPCASTITEMCOMP XsArray_compare,
	0
};

/*! \copydoc XsArray_constructDerived
	\note Specialization for XsStringArray
*/
void XsStringArray_construct(XsStringArray* thisPtr, XsSize count, XsString const* src)
{
	XsArray_construct(thisPtr, &g_xsStringArrayDescriptor, count, src);
}

/*! \relates XsStringArray
	\brief Splice the supplied string and put the resulting substrings in the string array
	\details The source string will be searched for instances of characters in \a separators and spliced whereever
	one was found. The spliced list will not include characters in \a separators and empty substrings will be
	discarded.
	\param src The source string to splice
	\param separators A list of separator characters that will be used to splice \a src.
*/
void XsStringArray_fromSplicedString(struct XsStringArray* thisPtr, struct XsString const* src, struct XsString const* separators)
{
	XsString s;
	XsString_construct(&s);
	XsArray_destruct(thisPtr);
	if (src->m_size > 0)
	{
		// check against 1 because an empty string can either be size 0 or size 1 (just the null-terminator)
		if (separators->m_size <= 1)
		{
			// no separator
			XsArray_insert(thisPtr, 0, 1, src);
		}
		else
		{
			char const* sep = (char const*) separators->m_data;
			char const* idx = (char const*) src->m_data;
			char const* newIdx = strpbrk(idx, sep);
			while (newIdx && *idx)
			{
				if (newIdx != idx)
				{
					XsString_assign(&s, (XsSize)(newIdx-idx), idx);
					XsArray_insert(thisPtr, thisPtr->m_size, 1, &s);
				}
				idx = newIdx+1;
				newIdx = strpbrk(idx, sep);
			}
			if (*idx)
			{
				XsString_assignCharArray(&s, idx);
				XsArray_insert(thisPtr, thisPtr->m_size, 1, &s);
			}
		}
	}
	XsString_destruct(&s);
}

/*! \relates XsStringArray
	\brief Join the string array into a single string, inserting \a separator between substrings.
	\param result The result of the join
	\param separator The separator to insert between successive items
*/
void XsStringArray_join(struct XsStringArray const* thisPtr, struct XsString* result, struct XsString const* separator)
{
	// determine required buffer size
	XsSize i;
	XsSize chars = (thisPtr->m_size ? (thisPtr->m_size-1) : 0) * (separator->m_size ? separator->m_size-1 : 0);
	for (i = 0; i < thisPtr->m_size; ++i)
	{
		XsSize sz = ((const XsString*)XsArray_at(thisPtr, i))->m_size;
		chars += (sz ? sz-1 : 0);
	}

	XsArray_destruct(result);
	if (chars)
	{
		XsArray_reserve(result, chars+1);
		for (i = 0; i < thisPtr->m_size; ++i)
		{
			const XsString* s = (const XsString*)XsArray_at(thisPtr, i);
			if (s->m_size > 1)
			{
				if (result->m_size > 1)
					XsString_append(result, separator);
				XsString_append(result, s);
			}
		}
	}
}
