
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
#define __LOC__ __FILE__ "(" __STR1__(__LINE__)") : WARNING: "

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
#	include <limits.h>
#	if __WORDSIZE == 64
#		define XSENS_64BIT
#	else
#		define XSENS_32BIT
#	endif
#	ifndef XSENS_PFSHARED
#		define XSENS_PFSHARED ".so"
#		define XSENS_PFPRE	  "lib"
#	endif
#else
#	ifndef XSENS_PFSHARED
#		define XSENS_PFSHARED ".dll"
#		define XSENS_PFPRE	  ""
#	endif
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
#	ifndef XSENS_PFBITS
#		ifdef __GNUC__
#			define XSENS_PFBITS ""
#		else
#			define XSENS_PFBITS	"64"
#		endif
#	endif
#else
#	ifndef XSENS_32BIT
#		define XSENS_32BIT
#	endif
#	ifndef XSENS_PFBITS
#		ifdef __GNUC__
#			define XSENS_PFBITS ""
#		else
#			define XSENS_PFBITS	"32"
#		endif
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
RelWithDeb    | MD      | Yes     | XSENS_DEBUG;XSENS_RELEASE;_DEBUG
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
	#ifndef XSENS_NO_STL
		#define XSENS_USE_DEBUG_COUNTERS
	#endif

	#if !defined(XSENS_RELEASE)
		//// Debug build
		#if !defined(QT_DEBUG) && !defined(QT_NO_DEBUG)
			#define QT_DEBUG	1
		#endif
		#define XSENS_CONFIG	0
		#ifndef XSENS_PFCONF
			#ifdef __GNUC__
				#define XSENS_PFCONF	""
			#else
				#define XSENS_PFCONF	"d"
			#endif
		#endif
	#else
		//// RelWithDeb build
		#if !defined(QT_DEBUG) && !defined(QT_NO_DEBUG)
			#define QT_NO_DEBUG	1
		#endif
		#define XSENS_CONFIG	1
		#ifndef XSENS_PFCONF
			#ifdef __GNUC__
				#define XSENS_PFCONF	""
			#else
				#define XSENS_PFCONF	"rd"
			#endif
		#endif
	#endif
#else
	//// Release build
	#if !defined(QT_DEBUG) && !defined(QT_NO_DEBUG)
		#define QT_NO_DEBUG	1
	#endif
	#define XSENS_CONFIG	3
	#ifndef NDEBUG
		#ifndef KEEP_ASSERTS
			#define NDEBUG		// make sure assertions and other debug options are compiled away by MSVC
		#endif
	#endif
	#ifndef XSENS_PFCONF
		#define XSENS_PFCONF	""
	#endif
#endif

#ifndef XSENS_PFFULL
	#define XSENS_PFFULL	XSENS_PFBITS XSENS_PFCONF
#endif

//////////////////////////////////////////////////
// more generic preprocessor defines
// required for gnu c++ compiler versions due to difference in attribute declarations
#if defined(__AVR32__)
#	define __cdecl
#	define __stdcall
#elif defined(_ADI_COMPILER)
#	define __cdecl
#	define __stdcall
#elif defined(__GNUC__) && !defined(HAVE_CDECL)
#	if !defined(__cdecl)
#		if defined(__x86_64__)
#			define __cdecl
#		else
#	define __cdecl __attribute__((cdecl))
#		endif
#	endif
#	if !defined(__stdcall)
#		if defined(__x86_64__)
#			define __stdcall
#		else
#	define __stdcall __attribute__((stdcall))
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
//#define XSENS_SINGLE_PRECISION // enable for single precision
#include "xstypedefs.h"

#ifndef XSNOCOMEXPORT
#define XSNOCOMEXPORT
#endif

#define XSTRINGIFY2(s)	#s
#define XSTRINGIFY(s)	XSTRINGIFY2(s)

#ifdef XSENS_DEBUG
#define XSDEBUGLINE(a)	a
#define XSNODEBUGLINE(...)
#else
#define XSDEBUGLINE(...)
#define XSNODEBUGLINE(a)	a
#endif

#if defined(_MSC_VER) && !defined(__cplusplus) && !defined(inline)
/* MSVC doesn't know the inline keyword in C mode, but it does know __inline */
#define inline __inline
#endif

#endif
