/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#define XSTYPES_DLL_API
#define XSNOEXPORT
#define XSENS_NO_AUTOLIB
// include this file in Visual Studio using C/C++->Advanced->Force Includes (the /FI option)
#ifndef XSTYPES_CONFIG_H
#define XSTYPES_CONFIG_H

//////////////////////////////////////////////////
// generic preprocessor defines

//http://support.microsoft.com/kb/155196
#define __STR2__(x) #x
#define __STR1__(x) __STR2__(x)
#define __LOC__ __FILE__ "(" __STR1__(__LINE__) ") : WARNING: "

// make sure both _WIN32 and WIN32 are defined if either of them is.
#if defined(_WIN32) || defined(_M_IX86)
#	ifndef WIN32
#		define WIN32
#	endif
#	define XSENS_WINDOWS
#endif

#ifdef WIN32
#	ifndef _WIN32
#		define _WIN32
#		define XSENS_WINDOWS
#	endif
#endif
#if !defined(_WIN32) && defined(_MSC_VER) && !defined(_WIN64)
#	define _WIN64
#endif

// make things as secure as possible without modifying the code...
#ifndef _CRT_SECURE_CPP_OVERLOAD_STANDARD_NAMES
#define _CRT_SECURE_CPP_OVERLOAD_STANDARD_NAMES 1
#endif
#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#endif

#ifndef __cplusplus
// make sure that const-correctness is enforced
#ifdef _MSC_VER
#pragma warning(error : 4090)
#endif
#define XSCCONST	const
#define XSCPPPROTECTED
#else
#define XSCCONST
#define XSCPPPROTECTED	protected:
#endif

#ifdef __GNUC__
#include <limits.h>
#if __WORDSIZE == 64
#	define XSENS_64BIT
#else
#	define XSENS_32BIT
#endif
#endif

#if defined(_WIN64) || defined(_M_X64) || defined(_M_IA64)
#	ifndef XSENS_64BIT
#		define XSENS_64BIT
#	endif
#	ifndef XSENS_WINDOWS
#		define XSENS_WINDOWS
#	endif
#	ifndef _WIN64
#		define _WIN64
#	endif
#	ifndef WIN64
#		define WIN64
#	endif
#else
#	ifndef XSENS_32BIT
#		define XSENS_32BIT
#	endif
#endif

// all xsens libraries should use unicode
#ifndef UNICODE
#define UNICODE
#endif

// use XSENS_32BIT and XSENS_64BIT to check for 32/64 bit builds in your application
// on non-windows systems these should be defined in this file

/*
Configuration | Runtime | DebInfo | Defines
--------------+---------------------------------------
Debug         | MDd     | Yes     | XSENS_DEBUG;_DEBUG
RelWithDeb    | MD      | Yes     | XSENS_DEBUG;XSENS_RELEASE;NDEBUG
Release       | MD      | No      | XSENS_RELEASE;NDEBUG

The common way to setup configuration-dependent defines:
#if defined(XSENS_DEBUG)
	//// Debug or RelWithDeb build
	#if defined(XSENS_RELEASE)
		//// RelWithDeb build
	#else
		//// Debug build
	#endif
#else
	//// Release build
#endif
*/

#if defined(XSENS_DEBUG)
	//// Debug or RelWithDeb build

	#if !defined(XSENS_RELEASE)
		//// Debug build
		#if !defined(QT_DEBUG) && !defined(QT_NO_DEBUG)
		#define QT_DEBUG	1
		#endif
		#define XSENS_CONFIG	0
	#else
		//// RelWithDeb build
		#if !defined(QT_DEBUG) && !defined(QT_NO_DEBUG)
		#define QT_NO_DEBUG	1
		#endif
		#define XSENS_CONFIG	1
	#endif
#else
	//// Release build
	#if !defined(QT_DEBUG) && !defined(QT_NO_DEBUG)
	#define QT_NO_DEBUG	1
	#endif
	#define XSENS_CONFIG	3
#endif

//////////////////////////////////////////////////
// more generic preprocessor defines
// required for gnu c++ compiler versions due to difference in attribute declarations
#if defined(__AVR32__)
#	define __cdecl
#	define __stdcall
#elif defined(_ADI_COMPILER)
#   define __cdecl
#   define __stdcall
#elif defined(__GNUC__) && !defined(HAVE_CDECL)
#	if !defined(__cdecl)
#		if defined(__x86_64__)
#			define __cdecl
#		else
#   define __cdecl __attribute__((cdecl))
#		endif
#	endif
#	if !defined(__stdcall)
#		if defined(__x86_64__)
#			define __stdcall
#		else
#   define __stdcall __attribute__((stdcall))
#endif
#	endif
#endif


//////////////////////////////////////////////////
// generic preprocessor defines

// use XSENS_32BIT and XSENS_64BIT to check for 32/64 bit builds in your application
// on non-windows systems these should be defined

#ifndef XSTYPES_DLL_API
#	ifdef XSTYPES_DLL_EXPORT
#		ifdef _WIN32
//#			pragma message("XSTYPES_DLL_API export in xstypesconfig.h")
#			define XSTYPES_DLL_API __declspec(dllexport)
#		else
//#			pragma message("XSTYPES_DLL_API linux export in xstypesconfig.h")
#			define XSTYPES_DLL_API __attribute__((visibility("default")))
#		endif
#	else	// ifdef XSTYPES_DLL_EXPORT
#		ifdef XSTYPES_STATIC_LIB
//#			pragma message("XSTYPES_DLL_API static in xstypesconfig.h")
#			define XSTYPES_DLL_API
#		else
#			ifdef _WIN32
//#				pragma message("XSTYPES_DLL_API import in xstypesconfig.h")
#				define XSTYPES_DLL_API __declspec(dllimport)
#			else
//#				pragma message("XSTYPES_DLL_API import/static for linux in xstypesconfig.h")
#				define XSTYPES_DLL_API
#			endif
#		endif
#	endif	// ifdef XSTYPES_DLL_EXPORT - else
#endif	// ifndef XSTYPES_DLL_API

#if XSENS_CONFIG < 3		// anything except full release builds
//////////////////////////////////////////////////
// stuff for debugging

#else
// non-debug stuff
//
#endif

#include <assert.h>

// since this is (almost) always required and it does not conflict with pstdint, include the file here
#include "xstypedefs.h"

#ifndef XSNOCOMEXPORT
#define XSNOCOMEXPORT
#endif

#endif	// file guard
