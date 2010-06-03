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

#ifndef ARTYPEDEFS_H
#define ARTYPEDEFS_H

#include <time.h>
#include <string>
#include <map>
#include <stdarg.h>
#include <list>

#ifdef WIN32

#ifndef SWIG
#if !defined(ARIA_STATIC) && !defined(AREXPORT)
#define AREXPORT _declspec(dllimport)
#elif !defined(AREXPORT) // ARIA_STATIC
#define AREXPORT
#endif // ARIA_STATIC
#else
#define AREXPORT
#endif

#include <winsock2.h>
#include <windows.h>

#endif //WIN32L


#ifndef WIN32

#define AREXPORT
////
//// Linux
////

#endif // linux


typedef std::map<int, std::string> ArStrMap;

/// has enum for position in list
class ArListPos
{
public:
  typedef enum {
      FIRST = 1, ///< place item first in the list
      LAST = 2 ///< place item last in the list
  } Pos;
};

/// Contains platform independent sized variable types
class ArTypes
{
public:
  /// A single signed byte
  typedef char Byte;
  /// Two signed bytes
  typedef short Byte2;
  /// Four signed bytes
  typedef int Byte4;

  /// A single unsigned byte
  typedef unsigned char UByte;
  /// Two unsigned bytes
  typedef unsigned short UByte2;
  /// Four unsigned bytes
  typedef unsigned int UByte4;
};


#endif
