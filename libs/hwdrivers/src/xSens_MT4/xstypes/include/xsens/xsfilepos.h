/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef XSFILEPOS_H
#define XSFILEPOS_H

/*! \addtogroup cinterface C Interface
	@{
*/

/*!	\typedef XsFilePos
	\brief The type that is used for positioning inside a file
*/
/*!	\typedef XsIoHandle
	\brief The type that is used for low-level identification of an open I/O device
*/
/*!	\typedef XsFileHandle
	\brief The type that is used for low-level identification of an open file
*/

/*! @} */

#include <stdio.h>
#ifdef _WIN32
#ifndef _PSTDINT_H_INCLUDED
#	include "pstdint.h"
#endif
typedef __int64 XsFilePos;
#ifndef HANDLE
#	include <windows.h>
#endif
typedef HANDLE XsIoHandle;
#else
#include <sys/types.h>
/* off_t is practically guaranteed not to be 64 bits on non64 bit systems.
   We'd better explicitly use __off64_t to be sure of it's size.
*/
#ifdef __APPLE__
typedef int64_t XsFilePos;
#else
typedef  __off64_t	XsFilePos;
#endif
typedef int32_t XsIoHandle;
#endif
typedef FILE XsFileHandle;

#endif	// file guard
