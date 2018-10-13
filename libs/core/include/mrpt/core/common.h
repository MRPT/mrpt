/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#pragma once

/* ------------------------------------
		  Disable some warnings
   ------------------------------------ */
#if defined(_MSC_VER)
#pragma warning(disable : 4127)  // Disable conditional expression is constant
// (it shows up in templates where it's
// correct)
#pragma warning(disable : 4244)  // argument conversion "possible loss of data"
#pragma warning(disable : 4503)  // (Compiler: Visual C++ 2010) Disable warning
// for too long decorated name
#pragma warning(disable : 4714)  // force inlined -> not inlined (in Eigen3)
#pragma warning(disable : 4267)  // truncation size_t -> type
#pragma warning(disable : 4290)  // Visual C++ does not implement decl.
// specifiers: throw(A,B,...)
#pragma warning(disable : 4251)  // Visual C++ 2003+ warnings on STL classes
// when exporting to DLL...
#pragma warning(disable : 4275)  // DLL export class derived from STL
#pragma warning( \
	disable : 4251)  // Warnings are emited even if a DLL export class
// contains a *private* STL field (?)
#if (_MSC_VER >= 1400)
// MS believes they have the right to deprecate functions in the C++ Standard
// STL... disable their warnings:
#ifndef _SCL_SECURE_NO_WARNINGS
#define _SCL_SECURE_NO_WARNINGS
#endif
// For the new secure library in VC++8
#if !defined(_CRT_SECURE_CPP_OVERLOAD_STANDARD_NAMES)
#define _CRT_SECURE_CPP_OVERLOAD_STANDARD_NAMES 1
#endif
#endif
#endif

// Avoid conflicting declaration of max macro in windows headers
#if defined(_WIN32) && !defined(NOMINMAX)
#define NOMINMAX
#ifdef max
#undef max
#undef min
#endif
#endif

// Generic constants and defines:
// ---------------------------------------------------------
// M_PI: Rely on standard <cmath>
#ifndef M_2PI
#define M_2PI 6.283185307179586476925286766559  // The 2*PI constant
#endif

#define M_PIf 3.14159265358979f
#define M_2PIf 6.28318530717959f

/**  MRPT_CHECK_GCC_VERSION(MAJ,MIN) */
#if defined(__GNUC__) && defined(__GNUC_MINOR__)
#define MRPT_CHECK_GCC_VERSION(major, minor) \
	((__GNUC__ > (major)) || (__GNUC__ == (major) && __GNUC_MINOR__ >= (minor)))
#else
#define MRPT_CHECK_GCC_VERSION(major, minor) 0
#endif

/** MRPT_CHECK_VISUALC_VERSION(Version) Version=8 for 2005, 9=2008, 10=2010,
 * 11=2012, 12=2013, 14=2015 */
#ifndef _MSC_VER
#define MRPT_VISUALC_VERSION(major) 0
#define MRPT_CHECK_VISUALC_VERSION(major) 0
#else
/* (From wxWidgets macros):
Things used to be simple with the _MSC_VER value and the version number
increasing in lock step, but _MSC_VER value of 1900 is VC14 and not the
non existing (presumably for the superstitious reasons) VC13, so we now
need to account for this with an extra offset.
*/
#define MRPT_VISUALC_VERSION(major) \
	((6 + (major >= 14 ? (-1) : 0) + major) * 100)
#define MRPT_CHECK_VISUALC_VERSION(major) \
	(_MSC_VER >= MRPT_VISUALC_VERSION(major))
#endif

#ifndef __has_feature
#define __has_feature(x) 0  // Compatibility with non-clang compilers.
#endif
#ifndef __has_extension
#define __has_extension __has_feature  // Compatibility with pre-3.0 compilers.
#endif

// A cross-compiler definition for "deprecated"-warnings
/** Usage: MRPT_DEPRECATED("Use XX instead") void myFunc(double); */
#if defined(__clang__) && defined(__has_extension)
#if __has_extension(attribute_deprecated_with_message)
#define MRPT_DEPRECATED(msg) __attribute__((deprecated(msg)))
#else
#define MRPT_DEPRECATED(msg) __attribute__((deprecated))
#endif
#elif MRPT_CHECK_GCC_VERSION(4, 5)
#define MRPT_DEPRECATED(msg) __attribute__((deprecated(msg)))
#elif MRPT_CHECK_VISUALC_VERSION(8)
#define MRPT_DEPRECATED(msg) __declspec(deprecated("deprecated: " msg))
#else
#define MRPT_DEPRECATED(msg)
#endif

/** Declare MRPT_TODO(message)  */
#if defined(_MSC_VER)
#define MRPT_DO_PRAGMA(x) __pragma(x)
#define __STR2__(x) #x
#define __STR1__(x) __STR2__(x)
#define __MSVCLOC__ __FILE__ "("__STR1__(__LINE__) ") : "
#define MRPT_MSG_PRAGMA(_msg) MRPT_DO_PRAGMA(message(__MSVCLOC__ _msg))
#elif defined(__GNUC__)
#define MRPT_DO_PRAGMA(x) _Pragma(#x)
#define MRPT_MSG_PRAGMA(_msg) MRPT_DO_PRAGMA(message(_msg))
#else
#define MRPT_DO_PRAGMA(x)
#define MRPT_MSG_PRAGMA(_msg)
#endif

#define MRPT_WARNING(x) MRPT_MSG_PRAGMA("Warning: " x)
#define MRPT_TODO(x) MRPT_MSG_PRAGMA("TODO: " x)

// Define a decl. modifier for printf-like format checks at compile time:
#ifdef __GNUC__
#define MRPT_printf_format_check(_FMT_, _VARARGS_) \
	__attribute__((__format__(__printf__, _FMT_, _VARARGS_)))
#else
#define MRPT_printf_format_check(_FMT_, _VARARGS_)
#endif
// Define a decl. modifier for scanf-like format checks at compile time:
#ifdef __GNUC__
#define MRPT_scanf_format_check(_FMT_, _VARARGS_) \
	__attribute__((__format__(__scanf__, _FMT_, _VARARGS_)))
#else
#define MRPT_scanf_format_check(_FMT_, _VARARGS_)
#endif

/** A macro for obtaining the name of the current function:  */
#if defined(_MSC_VER) && (_MSC_VER >= 1300)
#define __CURRENT_FUNCTION_NAME__ __FUNCTION__
#else
#define __CURRENT_FUNCTION_NAME__ __PRETTY_FUNCTION__
#endif

// Define a decl. modifier for printf-like format checks at compile time:
#ifdef __GNUC__
#define MRPT_printf_format_check(_FMT_, _VARARGS_) \
	__attribute__((__format__(__printf__, _FMT_, _VARARGS_)))
#else
#define MRPT_printf_format_check(_FMT_, _VARARGS_)
#endif

// Define a decl. modifier for scanf-like format checks at compile time:
#ifdef __GNUC__
#define MRPT_scanf_format_check(_FMT_, _VARARGS_) \
	__attribute__((__format__(__scanf__, _FMT_, _VARARGS_)))
#else
#define MRPT_scanf_format_check(_FMT_, _VARARGS_)
#endif

/** Tells the compiler we really want to inline that function */
#if (defined _MSC_VER) || (defined __INTEL_COMPILER)
#define MRPT_FORCE_INLINE __forceinline
#else
#define MRPT_FORCE_INLINE inline
#endif

/** Determines whether this is an X86 or AMD64 platform */
#if defined(__amd64__) || defined(__amd64) || defined(__x86_64__) || \
	defined(__x86_64) || defined(_M_AMD64) || defined(_M_X64) ||     \
	defined(__i386__) || defined(__i386) || defined(_M_I86) ||       \
	defined(i386) || defined(_M_IX86) || defined(_X86_)
#define MRPT_IS_X86_AMD64 1
#endif

/** Can be used to avoid "not used parameters" warnings from the compiler
 */
#define MRPT_UNUSED_PARAM(a) (void)(a)
