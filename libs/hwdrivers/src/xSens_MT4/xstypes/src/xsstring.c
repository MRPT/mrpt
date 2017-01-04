/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#include "xsstring.h"
#include <stdlib.h>
#include "xsatomicint.h"
#include <string.h>	// memcpy

/*! \struct XsString
	\brief A 0-terminated managed string of characters
	\details This structure uses XsArray to manage its internal data. 
	The C++ interface reports the size of the string excluding the terminating 0, but since the C version
	uses XsArray directly, its m_size member includes the terminating 0.
	When using the C version, make sure	that the 0 character is always preserved when manipulating the data.
	\sa XsArray
*/

//! \copydoc XsArrayDescriptor::itemSwap \note Specialization for char
void swapChar(char* a, char* b)
{
	char tmp = *a;
	*a = *b;
	*b = tmp;
}

//! \copydoc XsArrayDescriptor::itemCopy \note Specialization for char
void copyChar(char* to, char const* from)
{
	*to = *from;
}

//! \copydoc XsArrayDescriptor::itemCompare \note Specialization for char
int compareChar(char const* a, char const* b)
{
	if (*a < *b)
		return -1;
	if (*a > *b)
		return 1;
	return 0;
}

//! \brief Descriptor for XsInt64Array
XsArrayDescriptor const g_xsStringDescriptor = {
	//lint --e{64} ignore exact type mismatches here
	sizeof(char),	// const size_t itemSize;	//!< \protected Size of a single data element
	XSEXPCASTITEMSWAP swapChar,		// void (*itemSwap)(void* a, void* b);
	0,				// void (*itemConstruct)(void* e);
	XSEXPCASTITEMCOPY copyChar,		// void (*itemCopyConstruct)(void* e, void const* s);
	0,				// void (*itemDestruct)(void* e);
	XSEXPCASTITEMCOPY copyChar,		// void (*itemCopy)(void const* from, void* to);
	XSEXPCASTITEMCOMP compareChar		// int (*itemCompare)(void const* a, void const* b);
};

/*! \copydoc XsArray_construct
	\note Specialization for XsString
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
		count = strlen(src)+1;

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
#ifdef _MSC_VER
		int unicodeLength = lstrlenW( src ); // Convert all UNICODE characters
		int required = WideCharToMultiByte(CP_UTF8, 0, src, unicodeLength, NULL, 0, NULL, NULL);
		if (required != -1 && required > 0)
		{
			if ((XsSize)required+1 > thisPtr->m_reserved)
				XsArray_reserve(thisPtr, required+1);
			WideCharToMultiByte(CP_UTF8, 0, src, unicodeLength, thisPtr->m_data, required+1, NULL, NULL); //lint !e534
			thisPtr->m_data[required] = '\0';
			*((XsSize*) &thisPtr->m_size) = required+1;
			return;
		}
#else
		size_t required = wcstombs(0, src, 0);
		if (required != (size_t) -1 && required > 0)
		{
			if ((XsSize)required+1 > thisPtr->m_reserved)
				XsArray_reserve(thisPtr, required+1);
			wcstombs(thisPtr->m_data, src, required+1);	//lint !e534
			*((XsSize*) &thisPtr->m_size) = required+1;
			return;
		}
#endif
	}
	XsArray_assign(thisPtr, 0, 0);
}

//lint -save -e429
/*! \brief This function copies the contents of the object to a unicode wchar_t array
*/
XsSize XsString_copyToWCharArray(const XsString* thisPtr, wchar_t* dest, XsSize size)
{
#ifdef _MSC_VER
	return MultiByteToWideChar(CP_UTF8, 0, thisPtr->m_data, (int) thisPtr->m_size, dest, (int) size);
#else
	return mbstowcs(dest, thisPtr->m_data, size) + (dest?0:1);
#endif
}
//lint -restore
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
