/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef ARIAOSDEF_H
#define ARIAOSDEF_H

#if defined(_WIN32) && !defined(WIN32)
#define WIN32 _WIN32
#endif

#ifdef _MSC_VER // WIN32

////
//// Windows - Massage the windows compiler into working
////

// Turn off warning of usage of 'this' in
// constructor chaining
#pragma warning(disable:4355)

// Turn off warning about truncated identifiers which happens
// in debug builds of code using STL templatized stuff.
#pragma warning(disable:4786)

// Turn off warning about 'benign macro redef'.
#pragma warning(disable:4142)

// Turn off warning about loosing from the conversion to double.
#pragma warning(disable:4244)

// Turn off warning about forcing value to bool 'true' or 'false'.
#pragma warning(disable:4800)

// Turn off warning about using some standard C libraries that have been deprecated
// by MSVC. (e.g. they want you to use snprintf_s instead of snprintf, etc.)
#pragma warning(disable:4996)

//#include <string.h>
//#include <stdio.h>
//#include "windows.h"

#include "ariaTypedefs.h"

// Compatibility functions to help windows out.
inline int strcasecmp(const char *s1, const char *s2) 
                    {return _stricmp(s1, s2);}
inline int strncasecmp(const char *s1, const char *s2, size_t n) 
                    {return _strnicmp(s1, s2, n);}

#define snprintf _snprintf
#define vsnprintf _vsnprintf
#endif


#endif // ARIAOSDEF_H
