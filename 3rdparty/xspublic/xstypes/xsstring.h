
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

#ifndef XSSTRING_H
#define XSSTRING_H

#include "xstypesconfig.h"
#include "xsarray.h"
#ifndef XSENS_NO_WCHAR
#include <wchar.h>
#endif // XSENS_NO_WCHAR
#include <string.h>

#ifdef __cplusplus
#include <string>
#include <cstdlib>
#include <cstdio>
extern "C" {
#endif

extern XsArrayDescriptor const XSTYPES_DLL_API g_xsStringDescriptor;

#ifndef __cplusplus
#define XsString_INITIALIZER	XSARRAY_INITIALIZER(&g_xsStringDescriptor)
XSARRAY_STRUCT(XsString, char);
typedef struct XsString XsString;
// obsolete:
#define XSSTRING_INITIALIZER	XsString_INITIALIZER
#else
struct XsString;
#endif

XSTYPES_DLL_API void XsString_construct(XsString* thisPtr);
XSTYPES_DLL_API void XsString_destruct(XsString* thisPtr);
XSTYPES_DLL_API void XsString_assign(XsString* thisPtr, XsSize count, const char* src);
XSTYPES_DLL_API void XsString_assignCharArray(XsString* thisPtr, const char* src);
XSTYPES_DLL_API void XsString_resize(XsString* thisPtr, XsSize count);
XSTYPES_DLL_API void XsString_append(XsString* thisPtr, XsString const* other);
XSTYPES_DLL_API void XsString_erase(XsString* thisPtr, XsSize index, XsSize count);
XSTYPES_DLL_API void XsString_push_back(XsString* thisPtr, char c);
XSTYPES_DLL_API XsSize XsString_utf8Len(XsString const * thisPtr);
XSTYPES_DLL_API int XsString_endsWith(XsString const * thisPtr, XsString const* other, int caseSensitive);
XSTYPES_DLL_API int XsString_startsWith(XsString const * thisPtr, XsString const* other, int caseSensitive);
XSTYPES_DLL_API int XsString_contains(XsString const * thisPtr, XsString const* other, int caseSensitive, XsSize* offset);
XSTYPES_DLL_API int XsString_empty(XsString const * thisPtr);
XSTYPES_DLL_API void XsString_sort(XsString* thisPtr);
XSTYPES_DLL_API void XsString_reverse(XsString* thisPtr);
XSTYPES_DLL_API int XsString_findSubStr(XsString const* thisPtr, XsString const* needle);
XSTYPES_DLL_API void XsString_mid(XsString* thisPtr, XsString const* source, XsSize start, XsSize count);

#ifndef XSENS_NO_WCHAR
XSTYPES_DLL_API XsSize XsString_copyToWCharArray(const XsString* thisPtr, wchar_t* dest, XsSize size);
XSTYPES_DLL_API wchar_t XsString_utf8At(const XsString* thisPtr, XsSize index);
XSTYPES_DLL_API void XsString_assignWCharArray(XsString* thisPtr, const wchar_t* src);
XSTYPES_DLL_API void XsString_push_backWChar(XsString* thisPtr, wchar_t c);
#endif // XSENS_NO_WCHAR

#ifndef __cplusplus
// obsolete:
#define XsString_copy(thisPtr, copy)	XsArray_copy(copy, thisPtr)
#define XsString_swap(a, b)				XsArray_swap(a, b)
#endif
#ifdef __cplusplus
} // extern "C"
#endif

#ifdef __cplusplus
/* We need some special template implementations for strings to keep them 0-terminated
*/
// this typedef is not _always_ interpreted correctly by doxygen, hence the occasional function where we're NOT using it.
typedef XsArrayImpl<char, g_xsStringDescriptor, XsString> XsStringType;

/*! \brief Returns the number of items currently in the array, excluding the terminating 0
	\details This function returns the number of bytes in the underlying char array. This doesn't mean it returns the number of characters as XsString assumes the char* is a utf-8 encoded string.
	\returns The number of items currently in the array
	\sa reserved \sa setSize \sa resize \sa utf8Len
*/
template<> inline XsSize XsStringType::size() const
{
	return m_size?m_size-1:0;
}

//! \copydoc XsArray_reserve
template<> inline void XsStringType::reserve(XsSize count)
{
	if (count)
		XsArray_reserve(this, count+1);
	else
		XsArray_reserve(this, 0);
}

//! \brief Returns the reserved space in number of items
template<> inline XsSize XsStringType::reserved() const
{
	return m_reserved?m_reserved-1:0;
}

/*! \brief indexed data access operator */
template<> inline char& XsStringType::operator[] (XsSize index)
{
	assert(index < size());
	return *ptrAt(m_data, (ptrdiff_t) index);
}

/*! \brief Removes \a count items from the array starting at \a index. \param index The index of the first item to remove. \param count The number of items to remove. */
template<> inline void XsStringType::erase(XsSize index, XsSize count)
{
	XsString_erase((XsString*) this, index, count);
}

/*! \brief Insert \a count \a items at \a index in the string
	\param items The items to insert, may not be 0 unless count is 0
	\param index The index to use for inserting. Anything beyond the end of the string (ie. -1) will
	append to the actual end of the string.
	\param count The number of items to insert
*/
template<> inline void XsArrayImpl<char, g_xsStringDescriptor, XsString>::insert(char const* items, XsSize index, XsSize count)
{
	if (size())
	{
		if (index >= size())
			index = size();
		XsArray_insert(this, index, count, items);
	}
	else
		XsString_assign((XsString*) this, count, items);
}

/*! \brief Assign the characters in \a src to the string
	\details This function is resizes the string to fit \a count characters and copies those from \a src.
	\param count The number of characters in src. If this value is 0 and \a scr is not 0, the value is
				determined automatically by looking through src until a terminating 0 is found.
	\param src The source string to copy from. If this is 0 and \a count is not 0, the string will be
				filled with spaces instead.
	\sa XsString_assign
*/
template<> inline void XsArrayImpl<char, g_xsStringDescriptor, XsString>::assign(XsSize count, char const* src)
{
	XsString_assign((XsString*) this, count, src);
}

/*! \brief This function resizes the contained string to the desired size, while retaining its contents
	\param count The desired size of the string. This excludes the terminating 0 character.
	\sa XsString_resize
*/
template<> inline void XsStringType::resize(XsSize count)
{
	XsString_resize((XsString*) this, count);
}

/*! \brief Set the size of the array to \a count.
	\details The contents of the array after this operation are undefined.
	\param count The desired new size fo the array.
	\sa XsArray_assign \sa reserve \sa resize
*/
template<> inline void XsStringType::setSize(XsSize count)
{
	if (count != size())
		XsString_assign((XsString*) this, count, 0);
}

/*! \copydoc XsArray_append \sa XsArray_append */
template<> inline void XsArrayImpl<char, g_xsStringDescriptor, XsString>::append(const XsStringType& other)
{
	XsString_append((XsString*) this, (XsString const*) &other);
}

struct XsString : public XsStringType {
	//! \brief Constructs an XsString
	inline explicit XsString(XsSize sz = 0, char const* src = 0)
		 : XsStringType()
	{
		if (sz || src)
			XsString_assign(this, sz, src);
	}

	//! \brief Constructs an XsString as a copy of \a other
	inline XsString(const XsStringType& other)
		 : XsStringType(other)
	{
	}

	//! \brief Constructs an XsInt64Array that references the data supplied in \a ref
	inline explicit XsString(char* ref, XsSize sz, XsDataFlags flags /* = XSDF_None */)
		: XsStringType(ref, sz+1, flags)
	{
	}
#ifndef XSENS_NOITERATOR
	//! \brief Constructs an XsString with the list bound by the supplied iterators \a beginIt and \a endIt
	template <typename Iterator>
	inline explicit XsString(Iterator const& beginIt, Iterator const& endIt)
		: XsStringType(beginIt, endIt)
	{
	}
#endif

	//! \brief Construct an XsString from a 0-terminated character array
	inline XsString(char const* src)
		: XsStringType()
	{
		if (src && src[0])
			XsString_assignCharArray(this, src);
	}

#ifndef XSENS_NO_WCHAR
	//! \brief Construct an XsString from a 0-terminated character array
	inline XsString(wchar_t const* src)
		: XsStringType()
	{
		if (src && src[0])
			XsString_assignWCharArray(this, src);
	}
#endif

#ifndef XSENS_NO_STL
	//! \brief Construct an XsString from a std::string
	inline XsString(std::string const& src)
		: XsStringType()
	{
		if (!src.empty())
			XsString_assign(this, (XsSize)src.size()+1, src.c_str());
	}

	//! \brief Construct an XsString from a std::wstring
	inline XsString(std::wstring const& src)
		: XsStringType()
	{
		if (!src.empty())
			XsString_assignWCharArray(this, src.c_str());
	}
#endif // XSENS_NO_STL

	//! \brief Return the internal 0-terminated C-style string
	inline char* c_str()
	{
		static const char nullChar = 0;
		if (empty())
			return const_cast<char*>(&nullChar);
		try
		{
			return begin().operator ->();
		}
		catch(...)
		{
			return const_cast<char*>(&nullChar);
		}
	}

	//! \brief Return the internal 0-terminated C-style string
	inline char const* c_str() const
	{
		static const char nullChar = 0;
		if (empty())
			return &nullChar;
		try
		{
			return begin().operator ->();
		}
		catch(...)
		{
			return &nullChar;
		}
	}

#ifndef XSENS_NO_STL
	//! \brief Return the string as a std::string
	std::string toStdString() const
	{
		if (empty())
			return std::string();
		return std::string((char const*)begin().operator ->());
	}
#endif // XSENS_NO_STL

	//! \brief Return \a other appended to this, without modifying this
	XsString operator + (XsString const& other) const
	{
		XsString tmp;
		tmp.reserve(size()+other.size());
		tmp.append(*this);
		tmp.append(other);
		return tmp;
	}

#ifndef XSENS_NO_STL
	//! \brief Return the string as a std::wstring
	std::wstring toStdWString() const
	{
		if (empty())
			return std::wstring();
		size_t s = XsString_copyToWCharArray(this, NULL, 0);
		std::wstring w;
		w.resize(s-1);
		(void) XsString_copyToWCharArray(this, &w[0], (XsSize)s);
		return w;
	}
#endif // XSENS_NO_STL

	/*! \cond NODOXYGEN */
	using XsStringType::operator ==;
	using XsStringType::operator !=;
#ifndef XSENS_NOITERATOR
	using XsStringType::operator <<;
#endif
	/*! \endcond */

	//! \brief Return true if the contents of \a str are identical to this string
	bool operator == (char const* str) const
	{
		if (!str) return empty();
		return !strcmp(c_str(), str);
	}

	//! \brief Return true if the contents of \a str differ from this string
	inline bool operator != (char const* str) const
	{
		return !(*this == str);
	}

	//! \brief Append a character array to the string in a stream-like way
	inline XsString& operator << (char const* str)
	{
		if (str && str[0])
		{
			XsString const ref(const_cast<char*>(str), (XsSize)strlen(str), XSDF_None);
			append(ref);
		}
		return *this;
	}

	//! \brief Append an integer to the string in a stream-like way
	inline XsString& operator << (int i)
	{
		char buffer[32];
		append(XsString(buffer, (XsSize) (ptrdiff_t) std::sprintf(buffer, "%d", i), XSDF_None));
		return *this;
	}

	//! \brief Append another string to the string in a stream-like way
	inline XsString& operator << (XsString const& s)
	{
		append(s);
		return *this;
	}

	//! \brief Return true if the contents if \a str are greater then this string
	inline bool operator < (const XsString &str) const
	{
#ifdef XSENS_NO_STL
		return (strcmp(c_str(), str.c_str()) < 0);
#else
		return (this->toStdString() < str.toStdString());
#endif
	}

	//! \brief Return true if the contents if \a str are less then this string
	inline bool operator > (const XsString &str) const
	{
#ifdef XSENS_NO_STL
		return (strcmp(c_str(), str.c_str()) > 0);
#else
		return (this->toStdString() > str.toStdString());
#endif
	}

	//! \brief Append a character to the string
	inline XsString& push_back(char c)
	{
		XsString_push_back(this, c);
		return *this;
	}

	//! \brief Return true when the string is empty
	inline bool empty() const
	{
		return !!XsString_empty(this);
	}

	using XsStringType::append;

	//! \brief Append a char array to the string
	inline void append(char const* other)
	{
		if (other)
		{
			XsString tmp(other);
			XsString_append(this, &tmp);
		}
	}

#ifndef XSENS_NO_WCHAR
	//! \brief Append a wchar array to the string
	inline void append(wchar_t const* other)
	{
		if (other)
		{
			XsString tmp(other);
			XsString_append(this, &tmp);
		}
	}
#endif

	//! \returns the number of characters in a utf-8 encoded string
	inline XsSize utf8Len() const
	{
		return XsString_utf8Len(this);
	}

	/*! \brief Returns whether this string ends with \a other (case-insensitive!)
		\param other The string to match with the end of this string
		\param caseSensitive Whether to compare case sensitive or not (case insensitive is the default)
		\return true when the string ends with the given string
	*/
	inline bool endsWith(XsString const& other, bool caseSensitive = false) const
	{
		return 0 != XsString_endsWith(this, &other, caseSensitive?1:0);
	}

	/*! \brief Returns whether this string starts with \a other (case-insensitive!)
		\param other The string to match with the start of this string
		\param caseSensitive Whether to compare case sensitive or not (case insensitive is the default)
		\return true when the string starts with the given string
	*/
	inline bool startsWith(XsString const& other, bool caseSensitive = false) const
	{
		return 0 != XsString_startsWith(this, &other, caseSensitive?1:0);
	}

	/*! \brief Returns whether this string contains \a other (case-insensitive!)
		\param other The string to match with this string
		\param caseSensitive Whether to compare case sensitive or not (case insensitive is the default)
		\param offset When not null, this will be filled with the first index where \a other was found
		\return true when the string contains the given string
	*/
	inline bool contains(XsString const& other, bool caseSensitive = false, XsSize* offset = 0) const
	{
		return 0 != XsString_contains(this, &other, caseSensitive?1:0, offset);
	}

#ifndef XSENS_NO_WCHAR
	/*! \brief The decoded UTF-8 character at index \a index in the UTF-8 encoded string
		\details http://en.wikipedia.org/wiki/Utf-8#Description
		\param index The index of the character to return.
		\returns the decoded UTF-8 character at index \a index in the UTF-8 encoded string
	*/
	inline wchar_t utf8At(XsSize index) const
	{
		return XsString_utf8At(this, index);
	}

	//! \brief Append a unicode character to the string
	inline XsString& push_back(wchar_t c)
	{
		XsString_push_backWChar(this, c);
		return *this;
	}
#endif

	//! \brief Sort the characters in the string
	inline void sort()
	{
		XsString_sort(this);
	}

	/*! \brief Reverse the characters in the string
		\note This does not take into account utf-8 encoded characters
	*/
	inline void reverse()
	{
		XsString_reverse(this);
	}

	/*! \brief Find the first occurrence of \a needle in the string
		\param needle The string to find
		\return The offset of \a needle or -1 if it was not found
	*/
	int findSubStr(XsString const& needle) const
	{
		return XsString_findSubStr(this, &needle);
	}

	/*! \brief Return a substring of the string
		\details The function returns a copy of up to \a count characters from the string, starting at offset \a start
		\param start The offset of the first character to copy
		\param count The maximum number of characters to copy
		\return The requested substring
	*/
	XsString mid(XsSize start, XsSize count) const
	{
		XsString rv;
		XsString_mid(&rv, this, start, count);
		return rv;
	}
};

#ifndef XSENS_NO_STL
namespace std
{
	template<typename _CharT, typename _Traits>
	basic_ostream<_CharT, _Traits>& operator<<(basic_ostream<_CharT, _Traits>& o, XsString const& xs)
	{
		return (o << xs.toStdString());
	}
}
#endif

#endif

#endif
