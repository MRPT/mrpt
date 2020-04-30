
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

#include "xsstring.h"
#include <stdlib.h>
#include <string.h>	// memcpy
#include <ctype.h>

#if defined(WIN32)
#include <Windows.h>
#endif

/*! \struct XsString
	\brief A 0-terminated managed string of characters
	\details This structure uses XsArray to manage its internal data.
	The C++ interface reports the size of the string excluding the terminating 0, but since the C version
	uses XsArray directly, its m_size member includes the terminating 0.
	When using the C version, make sure	that the 0 character is always preserved when manipulating the data.
	\sa XsArray
*/

/*! \copydoc XsArrayDescriptor::itemSwap
	\note Specialization for char*/
void swapChar(char* a, char* b)
{
	char tmp = *a;
	*a = *b;
	*b = tmp;
}

/*! \copydoc XsArrayDescriptor::itemCopy
	\note Specialization for char*/
void copyChar(char* to, char const* from)
{
	*to = *from;
}

/*! \copydoc XsArrayDescriptor::itemCompare
	\note Specialization for char*/
int compareChar(void const* a, void const* b)
{
	if (*(char*)a < *(char*)b)
		return -1;
	if (*(char*)a > *(char*)b)
		return 1;
	return 0;
}

//! \brief Descriptor for XsInt64Array
XsArrayDescriptor const g_xsStringDescriptor = {
	sizeof(char),	// const size_t itemSize;	//!< \protected Size of a single data element
	XSEXPCASTITEMSWAP swapChar,		// void (*itemSwap)(void* a, void* b);
	0,				// void (*itemConstruct)(void* e);
	XSEXPCASTITEMCOPY copyChar,		// void (*itemCopyConstruct)(void* e, void const* s);
	0,				// void (*itemDestruct)(void* e);
	XSEXPCASTITEMCOPY copyChar,		// void (*itemCopy)(void const* from, void* to);
	XSEXPCASTITEMCOMP compareChar,		// int (*itemCompare)(void const* a, void const* b);
	XSEXPCASTRAWCOPY  XsArray_rawCopy	// void (*rawCopy)(void* to, void const* from, XsSize count, XsSize iSize)
};

/*! \brief Initializes the XsString object as an empty string
	\details This function initializes the object as an empty string.
*/
void XsString_construct(XsString* thisPtr)
{
	XsArray_construct(thisPtr, &g_xsStringDescriptor, 0, 0);
}

/*! \copydoc XsArray_destruct
	\note Specialization for XsString
*/
void XsString_destruct(XsString* thisPtr)
{
	XsArray_destruct(thisPtr);
}

/*! \copydoc XsArray_assign
	\note Specialization for XsString
*/
void XsString_assign(XsString* thisPtr, XsSize count, const char* src)
{
	if (!count && src)
		count = (XsSize)strlen(src)+1;

	if (src)
	{
		if (src[count-1])
		{
			XsArray_assign(thisPtr, count+1, 0);
			memcpy(thisPtr->m_data, src, count);
			thisPtr->m_data[count] = 0;
		}
		else
			XsArray_assign(thisPtr, count, src);
	}
	else
	{
		if (count)
		{
			XsArray_assign(thisPtr, count+1, 0);
			memset(thisPtr->m_data, ' ', count);
			thisPtr->m_data[count] = 0;
		}
		else
			XsArray_assign(thisPtr, 0, 0);
	}
}

/*! \brief This function determines the size of \a src and copies the contents to the object */
void XsString_assignCharArray(XsString* thisPtr, const char* src)
{
	XsString_assign(thisPtr, 0, src);
}

#ifndef XSENS_NO_WCHAR
/*! \brief This function determines the size of \a src and copies the contents to the object after
	converting it from a unicode string to a multibyte character string.
*/
void XsString_assignWCharArray(XsString* thisPtr, const wchar_t* src)
{
	if (src)
	{
#ifdef WIN32
		int unicodeLength = lstrlenW( src ); // Convert all UNICODE characters
		int required = WideCharToMultiByte(CP_UTF8, 0, src, unicodeLength, NULL, 0, NULL, NULL);
		if (required != -1 && required > 0)
		{
			XsSize reqv = (XsSize)(ptrdiff_t)required+1;
			if (reqv > thisPtr->m_reserved)
				XsArray_reserve(thisPtr, reqv);
			WideCharToMultiByte(CP_UTF8, 0, src, unicodeLength, thisPtr->m_data, required+1, NULL, NULL);
			thisPtr->m_data[required] = '\0';
			*((XsSize*) &thisPtr->m_size) = reqv;
			return;
		}
#else
		size_t required = wcstombs(0, src, 0);
		if (required != (size_t) -1 && required > 0)
		{
			if ((XsSize)required+1 > thisPtr->m_reserved)
				XsArray_reserve(thisPtr, required+1);
			wcstombs(thisPtr->m_data, src, required+1);
			*((XsSize*) &thisPtr->m_size) = required+1;
			return;
		}
#endif
	}
	XsArray_assign(thisPtr, 0, 0);
}

/*! \brief This function copies the contents of the object to a unicode wchar_t array
*/
XsSize XsString_copyToWCharArray(const XsString* thisPtr, wchar_t* dest, XsSize size)
{
#ifdef WIN32
	return (XsSize)(ptrdiff_t) MultiByteToWideChar(CP_UTF8, 0, thisPtr->m_data, (int)(ptrdiff_t) thisPtr->m_size, dest, (int)(ptrdiff_t) size);
#else
	return mbstowcs(dest, thisPtr->m_data, size) + (dest?0:1);
#endif
}

/*! \brief Append unicode character \a c to the string
	\param c The character to append
*/
void XsString_push_backWChar(XsString* thisPtr, wchar_t c)
{
	wchar_t buf[2] = { c, 0 };
	XsString tmp;

	XsString_construct(&tmp);
	XsString_assignWCharArray(&tmp, buf);
	XsString_append(thisPtr, &tmp);
	XsString_destruct(&tmp);
}
#endif // XSENS_NO_WCHAR

/*! \brief This function resizes the contained string to the desired size, while retaining its contents
	\param count The desired size of the string. This excludes the terminating 0 character.
	\sa XsArray_resize
*/
void XsString_resize(XsString* thisPtr, XsSize count)
{
	XsSize sz = thisPtr->m_size;
	XsArray_resize(thisPtr, count?count+1:0);
	if (count)
	{
		for (;sz < count; ++sz)
			thisPtr->m_data[sz] = ' ';
		thisPtr->m_data[count] = 0;
	}
}

/*! \brief This function concatenates the \a other to this
*/
void XsString_append(XsString* thisPtr, XsString const* other)
{
	if (other && other->m_size > 1)
	{
		// remove terminating null from this and append arrays
		XsArray_erase(thisPtr, thisPtr->m_size-1, 1);
		XsArray_append(thisPtr, other);
		if (thisPtr == other)
		{
			// add terminating null again
			static const char nullChar = 0;
			XsArray_insert(thisPtr, (XsSize) -1, 1, &nullChar);
		}
	}
}

/*! \copydoc XsArray_erase
	\note The function maintains the terminating 0 character
*/
void XsString_erase(XsString* thisPtr, XsSize index, XsSize count)
{
	if (index + count >= thisPtr->m_size)
	{
		if (index)
			XsArray_erase(thisPtr, index, (thisPtr->m_size-1)-index);
		else
			XsArray_erase(thisPtr, 0, thisPtr->m_size);
	}
	else
		XsArray_erase(thisPtr, index, count);
}

/*! \brief Append character \a c to the string
	\param c The character to append
*/
void XsString_push_back(XsString* thisPtr, char c)
{
	XsSize sz = thisPtr->m_size;
	if (!sz)
		sz = 1;
	XsString_resize(thisPtr, sz);
	thisPtr->m_data[sz-1] = c;
}

uint8_t const * advanceUtf8(const uint8_t* p)
{
	if ((*p & 0xC0) != 0xC0)
		++p;
	else
		if (*p & 0x20)
			if (*p & 0x10)
				if (*p & 0x08)
					if (*p & 0x04)
						p += 6;
					else
						p += 5;
				else
					p += 4;
			else
				p += 3;
		else
			p += 2;
	return p;
}

/*!	\brief Returns the number of characters in a UTF-8 encoded string
	\details http://en.wikipedia.org/wiki/Utf-8#Description
	\returns the number of characters in a UTF-8 encoded string
*/
XsSize XsString_utf8Len(XsString const * thisPtr)
{
	XsSize count = 0;
	uint8_t const * p = (uint8_t const*) thisPtr->m_data;

	if (!thisPtr || !thisPtr->m_data)
		return 0;

	while (*p != 0)
	{
		++count;
		p = advanceUtf8(p);
	}
	return count;
}

#ifndef XSENS_NO_WCHAR
int32_t shiftUtf8(int32_t t, uint8_t const* p, int bytes)
{
	int i;
	for (i = 0; i < bytes; ++i)
		t = (t << 6) | (p[i] & 0x3F);
	return t;
}

/*! \brief The decoded UTF-8 character at index \a index in the UTF-8 encoded string
	\details http://en.wikipedia.org/wiki/Utf-8#Description
	\param index The index of the character to return.
	\returns the decoded UTF-8 character at index \a index in the UTF-8 encoded string
*/
wchar_t XsString_utf8At(XsString const* thisPtr, XsSize index)
{
	int32_t t = 0;
	uint8_t const * p = (uint8_t const*) thisPtr->m_data;

	if (!thisPtr || !thisPtr->m_data)
		return 0;

	while (*p != 0 && index)
	{
		--index;
		p = advanceUtf8(p);
	}

	if (*p == 0)
		return 0;

	// translate!

	if ((*p & 0xC0) != 0xC0)
		t = (*p & 0x7F);
	else
		if (*p & 0x20)
			if (*p & 0x10)
				if (*p & 0x08)
					if (*p & 0x04)
						t = shiftUtf8(p[0] & 0x01, p+1, 5);
					else
						t = shiftUtf8(p[0] & 0x03, p+1, 4);
				else
					t = shiftUtf8(p[0] & 0x07, p+1, 3);
			else
				t = shiftUtf8(p[0] & 0x0F, p+1, 2);
		else
			t = shiftUtf8(p[0] & 0x1F, p+1, 1);
	return (wchar_t) t;
}
#endif

/*! \brief Returns whether this string ends with \a other
	\param other The string to match with the end of this string
	\param caseSensitive Whether to compare case sensitive or not
	\return true when the string ends with the given string
*/
int XsString_endsWith(XsString const * thisPtr, XsString const* other, int caseSensitive)
{
	const char* left;
	const char* right;

	// we can never find a bigger string than our own string
	if (thisPtr->m_size < other->m_size)
		return 0;

	// we always match an empty string
	if (other->m_size <= 1)
		return 1;

	left = thisPtr->m_data + thisPtr->m_size - other->m_size;
	right = other->m_data;

	if (caseSensitive)
		for (; *left == *right && *right; ++left, ++right);
	else
		for (; tolower((unsigned char)*left) == tolower((unsigned char)*right) && *right; ++left, ++right);

	if (!*right)
		return 1;

	return 0;
}

/*! \brief Returns whether this string starts with \a other
	\param other The string to match with the start of this string
	\param caseSensitive Whether to compare case sensitive or not
	\return true when the string starts with the given string
*/
int XsString_startsWith(XsString const * thisPtr, XsString const* other, int caseSensitive)
{
	const char* left = thisPtr->m_data;
	const char* right = other->m_data;

	// we can never find a bigger string than our own string
	if (thisPtr->m_size < other->m_size)
		return 0;

	// we always match an empty string
	if (other->m_size <= 1)
		return 1;

	if (caseSensitive)
		for (; *left == *right && *right; ++left, ++right);
	else
		for (; tolower((unsigned char)*left) == tolower((unsigned char)*right) && *right; ++left, ++right);

	if (!*right)
		return 1;

	return 0;
}

/*! \brief Returns whether this string contains \a other
	\param other The string to match with this string
	\param caseSensitive Whether to compare case sensitive or not (case insensitive is the default)
	\param offset when not null, this will be filled with the offset at which \a other was found
	\return true when the string contains the given string
*/
int XsString_contains(XsString const * thisPtr, XsString const* other, int caseSensitive, XsSize* offset)
{
	XsSize offsetI = 0;
	if (!offset)
		offset = &offsetI;
	*offset = 0;

	// we always match an empty string
	if (other->m_size <= 1)
		return 1;

	// we can never find a bigger string than our own string
	while (thisPtr->m_size-*offset >= other->m_size)
	{
		const char* left = thisPtr->m_data+*offset;
		const char* right = other->m_data;
		if (caseSensitive)
			for (; *left == *right && *right; ++left, ++right);
		else
			for (; tolower((unsigned char)*left) == tolower((unsigned char)*right) && *right; ++left, ++right);

		if (!*right)
			return 1;

		++*offset;
	}
	*offset = (XsSize)-1;
	return 0;
}

/*! \brief Returns true when the supplied string is empty
	\return true when the string is empty
*/
int XsString_empty(XsString const * thisPtr)
{
	if (!thisPtr)
		return 1;
	if (!thisPtr->m_size || (thisPtr->m_flags & XSDF_Empty))
		return 1;
	return !(thisPtr->m_size-1);
}

/*! \brief Sorts the string
	\details This function sorts using qsort
*/
void XsString_sort(XsString* thisPtr)
{
	if (thisPtr->m_size > 2)
		qsort(thisPtr->m_data, thisPtr->m_size-1, sizeof(char), compareChar);
}

/*! \brief Reverses the contents of the string
	\details This reverses the contents in-place
	\note This does not take into account utf-8 encoded characters
*/
void XsString_reverse(XsString* thisPtr)
{
	int i, half;
	char* data, tmp, *right;
	if (thisPtr->m_size > 2)
	{
		half = (int) ((thisPtr->m_size-1) >> 1);
		data = (char*) thisPtr->m_data;
		right = data + thisPtr->m_size-2;
		for (i = 0; i < half; ++i)
		{
			tmp = data[i];
			data[i] = right[-i];
			right[-i] = tmp;
		}
	}
}

/*! \brief Find the first occurrence of \a needle in the string
	\param needle The string to find
	\return The offset of \a needle or -1 if it was not found
*/
int XsString_findSubStr(XsString const* thisPtr, XsString const* needle)
{
	XsSize offset, i, end, endN;
	if (!thisPtr)	// no string to search in
		return -1;
	if (!needle || needle->m_size <= 1)	// empty string matches start of string, even if searchee is empty
		return 0;
	if (thisPtr->m_size <= 1 || thisPtr->m_size < needle->m_size)
		return -1;

	end = thisPtr->m_size - needle->m_size;
	endN = needle->m_size-1;

	for (offset = 0; offset <= end; ++offset)
	{
		for (i = 0; i < endN; ++i)
			if (thisPtr->m_data[offset + i] != needle->m_data[i])
				break;
		if (i == endN)
			return (int) offset;	// found!
	}
	// not found
	return -1;
}

/*! \brief Copy a substring of the \a source string
	\details The function copies up to \a count characters from \a source to the string, starting at offset \a start
	\param source The source to copy from
	\param start The offset of the first character to copy
	\param count The maximum number of characters to copy
*/
void XsString_mid(XsString* thisPtr, XsString const* source, XsSize start, XsSize count)
{
	if (!thisPtr || !source)
		return;
	if (start >= source->m_size)
	{
		XsString_assign(thisPtr, 0, 0);
		return;
	}
	if (start + count >= source->m_size)
		count = (source->m_size - 1) - start;
	XsString_assign(thisPtr, count, count ? source->m_data + start : 0);
}
