/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef XSSTRING_H
#define XSSTRING_H

#include "xstypesconfig.h"
#include "xsarray.h"
#ifndef XSENS_NO_WCHAR
#include <wchar.h>
	#if !defined(XSENS_NO_STL) && defined(__cplusplus) && defined(WIN32)
	#include <Windows.h>	// required for MultiByteToWideChar
	#endif
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
#ifndef XSENS_NO_WCHAR
XSTYPES_DLL_API void XsString_assignWCharArray(XsString* thisPtr, const wchar_t* src);
#endif // XSENS_NO_WCHAR
XSTYPES_DLL_API XsSize XsString_copyToWCharArray(const XsString* thisPtr, wchar_t* dest, XsSize size);
//XSTYPES_DLL_API void XsString_assignWString(XsString* thisPtr, XsWString const* src);
XSTYPES_DLL_API void XsString_resize(XsString* thisPtr, XsSize count);
XSTYPES_DLL_API void XsString_append(XsString* thisPtr, XsString const* other);
XSTYPES_DLL_API void XsString_erase(XsString* thisPtr, XsSize index, XsSize count);

#ifndef __cplusplus
// obsolete:
#define XsString_copy(thisPtr, copy)	XsArray_copy(copy, thisPtr)
#define XsString_swap(a, b)				XsArray_swap(a, b)
#else
} // extern "C"
#endif

#ifdef __cplusplus
/* We need some special template implementations for strings to keep them 0-terminated
*/
// this typedef is not _always_ interpreted correctly by doxygen, hence the occasional function where we're NOT using it.
//template <typename T, XsArrayDescriptor const& D, typename I>
//typedef XsArrayImpl<char, g_xsStringDescriptor, XsString> XsStringType;  // MRPT: Converted into a struct to fix building with GCC 4.9+
struct XsStringType : public XsArrayImpl<char, g_xsStringDescriptor, XsString>
{
    inline XsStringType() : XsArrayImpl()
    {
    }

    inline explicit XsStringType(char* ref, XsSize sz, XsDataFlags flags) :
        XsArrayImpl(ref,sz,flags)
    {
    }

    /*! \brief Returns the number of items currently in the array, excluding the terminating 0
        \returns The number of items currently in the array
        \sa reserved \sa setSize \sa resize
    */
    inline XsSize size() const
    {
        return m_size?m_size-1:0;
    }

    //! \copydoc XsArray_reserve
    inline void reserve(XsSize count)
    {
        XsArray_reserve(this, count+1);
    }

    //! \brief Returns the reserved space in number of items
    inline XsSize reserved() const
    {
        return m_reserved?m_reserved-1:0;
    }

    /*! \brief indexed data access operator */
    inline char& operator[] (XsSize index)
    {
        assert(index < size());
        return *ptrAt(m_data, index);
    }

    /*! \brief Removes \a count items from the array starting at \a index. \param index The index of the first item to remove. \param count The number of items to remove. */
    inline void erase(XsSize index, XsSize count)
    {
        XsString_erase((XsString*) this, index, count);
    }

    /*! \brief Insert \a count \a items at \a index in the string
        \param items The items to insert, may not be 0 unless count is 0
        \param index The index to use for inserting. Anything beyond the end of the string (ie. -1) will
        append to the actual end of the string.
        \param count The number of items to insert
    */
    inline void insert(char const* items, XsSize index, XsSize count)
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
    inline void assign(XsSize count, char const* src)
    {
        XsString_assign((XsString*) this, count, src);
    }

    /*! \brief This function resizes the contained string to the desired size, while retaining its contents
        \param count The desired size of the string. This excludes the terminating 0 character.
        \sa XsString_resize
    */
    inline void resize(XsSize count)
    {
        XsString_resize((XsString*) this, count);
    }

    /*! \brief Set the size of the array to \a count.
        \details The contents of the array after this operation are undefined.
        \param count The desired new size fo the array.
        \sa XsArray_assign \sa reserve \sa resize
    */
    inline void setSize(XsSize count)
    {
        if (count != size())
            XsString_assign((XsString*) this, count, 0);
    }

    /*! \copydoc XsArray_append \sa XsArray_append */
    inline void append(const XsStringType& other)
    {
        XsString_append((XsString*) this, (XsString const*) &other);
    }
}; // end of struct XsStringType (MRPT)



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
	inline explicit XsString(char* ref, XsSize sz, XsDataFlags flags = XSDF_None)
		: XsStringType(ref, sz+1, flags)
	{
	}
#ifndef XSENS_NOITERATOR
	//! \brief Constructs an XsString with the list bound by the supplied iterators \a beginIt and \a endIt
	template <typename Iterator>
	inline XsString(Iterator const& beginIt, Iterator const& endIt)
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
			XsString_assign(this, src.size()+1, src.c_str());
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
			return (char*) &nullChar;
		return begin().operator ->();
	}

	//! \brief Return the internal 0-terminated C-style string
	inline char const* c_str() const
	{
		static const char nullChar = 0;
		if (empty())
			return &nullChar;
		return begin().operator ->();
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
		s = XsString_copyToWCharArray(this, &w[0], s);
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
			XsString const ref(const_cast<char*>(str), strlen(str), XSDF_None);
			append(ref);
		}
		return *this;
	}

	//! \brief Append an integer to the string in a stream-like way
	inline XsString& operator << (int i)
	{
		char buffer[32];
		append(XsString(buffer, sprintf(buffer, "%d", i), XSDF_None));
		return *this;
	}

	//! \brief Append another string to the string in a stream-like way
	inline XsString& operator << (XsString const& s)
	{
		append(s);
		return *this;
	}
};

#ifndef XSENS_NO_STL
namespace std {
template<typename _CharT, typename _Traits>
basic_ostream<_CharT, _Traits>& operator<<(basic_ostream<_CharT, _Traits>& o, XsString const& xs)
{
	return (o << xs.toStdString());
}
}
#endif

#endif

#endif // file guard
