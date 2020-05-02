
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

#define XSENS_NO_AUTOLIB
// include this file in Visual Studio using C/C++->Advanced->Force Includes (the /FI option)
#ifndef XSCONTROLLER_CONFIG_H
#define XSCONTROLLER_CONFIG_H

// define to build with Journaller
#ifndef HAVE_JOURNALLER
//#define HAVE_JOURNALLER
#endif

//////////////////////////////////////////////////
// generic preprocessor defines

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

#ifdef _WIN32
#	define USE_WINUSB
#else
#	define XSENS_NO_PORT_NUMBERS
#endif

// make things as secure as possible without modifying the code...
#ifndef _CRT_SECURE_CPP_OVERLOAD_STANDARD_NAMES
#define _CRT_SECURE_CPP_OVERLOAD_STANDARD_NAMES 1
#endif
#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
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

#ifndef XDA_DLL_API
#	ifdef XDA_DLL_EXPORT
#		ifdef _WIN32
//#			pragma message("XDA_DLL_API export in xscontrollerconfig.h")
#			define XDA_DLL_API __declspec(dllexport)
#		else
//#			pragma message("XDA_DLL_API linux export in xscontrollerconfig.h")
#			define XDA_DLL_API __attribute__((visibility("default")))
#		endif
#	else	// ifdef XDA_DLL_EXPORT
#		ifdef XDA_STATIC_LIB
//#			pragma message("XDA_DLL_API static in xscontrollerconfig.h")
#			define XDA_DLL_API
#		else
#			ifdef _WIN32
//#				pragma message("XDA_DLL_API import in xscontrollerconfig.h")
#				define XDA_DLL_API __declspec(dllimport)
#			else
//#				pragma message("XDA_DLL_API import/static for linux in xscontrollerconfig.h")
#				define XDA_DLL_API
#			endif
#		endif
#	endif	// ifdef XDA_DLL_EXPORT - else
#endif	// ifndef XDA_DLL_API

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

//////////////////////////////////////////////////
// more generic preprocessor defines
//! Add this macro to the start of a class definition to prevent automatic creation of copy functions
#define XSENS_DISABLE_COPY(className) \
private: \
	className(const className &); \
	className &operator = (const className &);

#ifdef __cplusplus
#ifdef HAVE_JOURNALLER
#include <xscommon/journaller.h>
#else
class Journaller;
#endif
extern Journaller* gJournal;
#endif

#if defined(HAVE_JOURNALLER) && (defined(XSENS_DEBUG) || defined(XS_LOG_ALWAYS))
#	define XSEXITLOGC(j)	JournalValueJanitor<XsResultValue> _xsExitLogC(j, m_lastResult, std::string(__FUNCTION__) + " exit, lastResult = ")
#	define XSEXITLOGD(j)	XSEXITLOGC(j)	//JournalValueJanitor<int> _xsExitLogD(j, d->m_lastResult, std::string(__FUNCTION__) + " exit, lastResult = ")
#	define XSEXITLOGN(j)	JournalValueJanitor<std::string> _xsExitLogN(j, std::string(), std::string(__FUNCTION__) + " exit")
#	ifndef DIDLOG
#		define DIDLOG(d)		JLHEXLOG_BARE(d)
#	endif
#else
#	define XSEXITLOGC(j)	((void) 0)
#	define XSEXITLOGD(j)	((void) 0)
#	define XSEXITLOGN(j)	((void) 0)
#	ifndef DIDLOG
#		define DIDLOG(d)		""
#	endif

#if !defined(HAVE_JOURNALLER) && !defined(JLDEBUG)
#define JLTRACE(...)		((void)0)
#define JLTRACE_NODEC(...)	((void)0)
#define JLTRACEG(...)		((void)0)
#define JLDEBUG(...)		((void)0)
#define JLDEBUG_NODEC(...)	((void)0)
#define JLDEBUGG(...)		((void)0)
#define JLALERT(...)		((void)0)
#define JLALERT_NODEC(...)	((void)0)
#define JLALERTG(...)		((void)0)
#define JLERROR(...)		((void)0)
#define JLERROR_NODEC(...)	((void)0)
#define JLERRORG(...)		((void)0)
#define JLFATAL(...)		((void)0)
#define JLFATAL_NODEC(...)	((void)0)
#define JLFATALG(...)		((void)0)
#define JLWRITE(...)		((void)0)
#define JLWRITE_NODEC(...)	((void)0)
#define JLWRITEG(...)		((void)0)

#endif

#endif

#ifndef XSNOEXPORT
#define XSNOEXPORT
#endif
#ifndef XSNOLINUXEXPORT
#define XSNOLINUXEXPORT
#endif

#endif
