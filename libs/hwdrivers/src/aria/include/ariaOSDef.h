/*
MobileRobots Advanced Robotics Interface for Applications (ARIA)
Copyright (C) 2004, 2005 ActivMedia Robotics LLC
Copyright (C) 2006, 2007 MobileRobots Inc.

     This program is free software; you can redistribute it and/or modify
     it under the terms of the GNU General Public License as published by
     the Free Software Foundation; either version 2 of the License, or
     (at your option) any later version.

     This program is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
     GNU General Public License for more details.

     You should have received a copy of the GNU General Public License
     along with this program; if not, write to the Free Software
     Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

If you wish to redistribute ARIA under different terms, contact 
MobileRobots for information about a commercial version of ARIA at 
robots@mobilerobots.com or 
MobileRobots Inc, 19 Columbia Drive, Amherst, NH 03031; 800-639-9481
*/

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
