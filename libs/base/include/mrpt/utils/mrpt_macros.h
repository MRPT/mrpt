/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef MRPT_MACROS_H
#define MRPT_MACROS_H

#include <mrpt/base/link_pragmas.h>
#include <sstream> // ostringstream
#include <stdexcept> // logic_error

/**  MRPT_CHECK_GCC_VERSION(MAJ,MIN) */
#if defined(__GNUC__) && defined(__GNUC_MINOR__)
#	define MRPT_CHECK_GCC_VERSION( major, minor ) ( ( __GNUC__ > (major) )  || ( __GNUC__ == (major) && __GNUC_MINOR__ >= (minor) ) )
#else
#	define MRPT_CHECK_GCC_VERSION( major, minor ) 0
#endif

/** MRPT_CHECK_VISUALC_VERSION(Version) Version=8 for 2005, 9=2008, 10=2010, 11=2012, 12=2013, 14=2015 */
#ifndef _MSC_VER
#   define MRPT_VISUALC_VERSION(major) 0
#   define MRPT_CHECK_VISUALC_VERSION(major) 0
#else
   /* (From wxWidgets macros):
   Things used to be simple with the _MSC_VER value and the version number
   increasing in lock step, but _MSC_VER value of 1900 is VC14 and not the
   non existing (presumably for the superstitious reasons) VC13, so we now
   need to account for this with an extra offset.
   */
#   define MRPT_VISUALC_VERSION(major) ( (6 + (major >= 14 ? (-1) : 0) + major) * 100 )
#   define MRPT_CHECK_VISUALC_VERSION(major) ( _MSC_VER >= MRPT_VISUALC_VERSION(major) )
#endif

#ifndef __has_feature
#	define __has_feature(x) 0  // Compatibility with non-clang compilers.
#endif
#ifndef __has_extension
#	define __has_extension __has_feature // Compatibility with pre-3.0 compilers.
#endif


/** Does the compiler support C++11? */
#if (__cplusplus>199711L || MRPT_CHECK_VISUALC_VERSION(11) )
#	define MRPT_HAS_CXX11  1
#else
#	define MRPT_HAS_CXX11  0
#endif

/** C++11 "override" for virtuals: */
#if MRPT_HAS_CXX11
#	define MRPT_OVERRIDE  override
#else
#	define MRPT_OVERRIDE
#endif

/** C++11 deleted function declarations */
#if MRPT_CHECK_VISUALC_VERSION(12) || __has_extension(cxx_deleted_functions) || (MRPT_CHECK_GCC_VERSION(4,4) && MRPT_HAS_CXX11)
#	define MRPT_DELETED_FUNC   =delete
#else
#	define MRPT_DELETED_FUNC
#endif

/** C++11 noexcept: Used after member declarations */
#if MRPT_CHECK_VISUALC_VERSION(14) || __has_extension(cxx_noexcept) || (MRPT_CHECK_GCC_VERSION(4,6) && MRPT_HAS_CXX11)
#	define MRPT_NO_THROWS noexcept
#else
#	define MRPT_NO_THROWS  throw()
#endif

/** C++11 unique_ptr<> */
#if MRPT_CHECK_VISUALC_VERSION(10) || (MRPT_CHECK_GCC_VERSION(4,4) || defined(__clang__)) && (MRPT_HAS_CXX11 || defined(__GXX_EXPERIMENTAL_CXX0X__))
#	define MRPT_HAS_UNIQUE_PTR  1
#else
#	define MRPT_HAS_UNIQUE_PTR  0
#endif

// A cross-compiler definition for "deprecated"-warnings
/** Usage: MRPT_DEPRECATED("Use XX instead") void myFunc(double); */
#if defined(__clang__) && defined(__has_extension)
#	if __has_extension(attribute_deprecated_with_message)
#		define MRPT_DEPRECATED(msg) __attribute__((deprecated(msg)))
#	else
#		define MRPT_DEPRECATED(msg) __attribute__((deprecated))
#	endif
#elif MRPT_CHECK_GCC_VERSION(4, 5)
#	define MRPT_DEPRECATED(msg) __attribute__((deprecated(msg)))
#elif MRPT_CHECK_VISUALC_VERSION(8)
#	define MRPT_DEPRECATED(msg) __declspec(deprecated("deprecated: " msg))
#else
#	define MRPT_DEPRECATED(msg) 
#endif


/** Declare MRPT_TODO(message)  */
#if defined(_MSC_VER)
	#define MRPT_DO_PRAGMA(x) __pragma(x)
	#define __STR2__(x) #x
	#define __STR1__(x) __STR2__(x)
	#define __MSVCLOC__ __FILE__ "("__STR1__(__LINE__)") : "
	#define MRPT_MSG_PRAGMA(_msg) MRPT_DO_PRAGMA(message (__MSVCLOC__ _msg))
#elif defined(__GNUC__)
	#define MRPT_DO_PRAGMA(x) _Pragma (#x)
	#define MRPT_MSG_PRAGMA(_msg) MRPT_DO_PRAGMA(message (_msg))
#else
	#define MRPT_DO_PRAGMA(x)
	#define MRPT_MSG_PRAGMA(_msg)
#endif

#define MRPT_WARNING(x) MRPT_MSG_PRAGMA("Warning: " x)
#define MRPT_TODO(x)	MRPT_MSG_PRAGMA("TODO: " x)

// Define a decl. modifier for printf-like format checks at compile time:
#ifdef __GNUC__
#	define MRPT_printf_format_check(_FMT_,_VARARGS_)  __attribute__ ((__format__ (__printf__, _FMT_,_VARARGS_)))
#else
#	define MRPT_printf_format_check(_FMT_,_VARARGS_)
#endif
// Define a decl. modifier for scanf-like format checks at compile time:
#ifdef __GNUC__
#	define MRPT_scanf_format_check(_FMT_,_VARARGS_)  __attribute__ ((__format__ (__scanf__, _FMT_,_VARARGS_)))
#else
#	define MRPT_scanf_format_check(_FMT_,_VARARGS_)
#endif

// A cross-compiler definition for aligned memory structures:
#if defined(_MSC_VER)
	#define MRPT_ALIGN16 __declspec(align(16))
	#define MRPT_ALIGN32 __declspec(align(32))
#elif defined(__GNUC__)
	#define MRPT_ALIGN16 __attribute__((aligned(16)))
	#define MRPT_ALIGN32 __attribute__((aligned(32)))
#else
	#define MRPT_ALIGN16
	#define MRPT_ALIGN32
#endif

/** A macro for obtaining the name of the current function:  */
#if defined(_MSC_VER) && (_MSC_VER>=1300)
		#define	__CURRENT_FUNCTION_NAME__	__FUNCTION__
#else
		#define	__CURRENT_FUNCTION_NAME__	__PRETTY_FUNCTION__
#endif

/** \def THROW_EXCEPTION(msg)
 * \param msg This can be a char*, a std::string, or a literal string.
 * Defines a unified way of reporting exceptions
 * \sa MRPT_TRY_START, MRPT_TRY_END, THROW_EXCEPTION_CUSTOM_MSG1
 */
#define THROW_EXCEPTION(msg)	\
	{\
		std::ostringstream auxCompStr;\
		auxCompStr << "\n\n =============== MRPT EXCEPTION =============\n";\
		auxCompStr << __CURRENT_FUNCTION_NAME__ << ", line " << __LINE__ << ":\n";\
		auxCompStr << msg << std::endl; \
		throw std::logic_error( auxCompStr.str() );\
	}\

/** \def THROW_EXCEPTION_CUSTOM_MSG1
  * \param e The caught exception.
  *	\param msg Is a char* or literal string.
  */
#define THROW_EXCEPTION_CUSTOM_MSG1(msg,param1)	\
	{\
		std::ostringstream auxCompStr;\
		auxCompStr << "\n\n =============== MRPT EXCEPTION =============\n";\
		auxCompStr << __CURRENT_FUNCTION_NAME__ << ", line " << __LINE__ << ":\n";\
		auxCompStr << mrpt::format(msg,param1)<< std::endl; \
		throw std::logic_error( auxCompStr.str() );\
	}\


/** \def THROW_TYPED_EXCEPTION(msg,exceptionClass)
 * Defines a unified way of reporting exceptions of type different from "std::exception"
 * \sa MRPT_TRY_START, MRPT_TRY_END
 */
#define THROW_TYPED_EXCEPTION(msg,exceptionClass)	\
	{\
		std::ostringstream auxCompStr;\
		auxCompStr << "\n\n =============== MRPT EXCEPTION =============\n";\
		auxCompStr << __CURRENT_FUNCTION_NAME__ << ", line " << __LINE__ << ":\n";\
		auxCompStr << msg << std::endl; \
		throw exceptionClass( auxCompStr.str() );\
	}\

/** \def THROW_EXCEPTION_CUSTOM_MSG1
  * \param e The caught exception.
  *	\param msg Is a char* or literal string.
  */
#define THROW_TYPED_EXCEPTION_CUSTOM_MSG1(msg,param1,exceptionClass)	\
	{\
		std::ostringstream auxCompStr;\
		auxCompStr << "\n\n =============== MRPT EXCEPTION =============\n";\
		auxCompStr << __CURRENT_FUNCTION_NAME__ << ", line " << __LINE__ << ":\n";\
		auxCompStr << mrpt::format(msg,param1)<< std::endl; \
		throw exceptionClass( auxCompStr.str() );\
	}\


/** \def THROW_STACKED_EXCEPTION
 * \sa MRPT_TRY_START, MRPT_TRY_END
 */
#define THROW_STACKED_EXCEPTION(e)	\
	{\
		std::string _tse_str( e.what() );\
		_tse_str+= __CURRENT_FUNCTION_NAME__;\
		_tse_str+= mrpt::format(", line %i:\n", __LINE__ );\
		throw std::logic_error( _tse_str );\
	}

/** \def THROW_STACKED_EXCEPTION_CUSTOM_MSG
  * \param e The caught exception.
  *	\param msg Is a char* or std::string.
  */
#define THROW_STACKED_EXCEPTION_CUSTOM_MSG1(e,msg)	\
	{\
		std::ostringstream auxCompStr;\
		auxCompStr << e.what() ;  \
		auxCompStr << msg << std::endl; \
		throw std::logic_error( auxCompStr.str() );\
	}\

/** \def THROW_STACKED_EXCEPTION_CUSTOM_MSG
  * \param e The caught exception.
  *	\param stuff Is a printf-like sequence of params, e.g: "The error happens for x=%i",x
  */
#define THROW_STACKED_EXCEPTION_CUSTOM_MSG2(e,stuff,param1)	\
	{\
		std::ostringstream auxCompStr;\
		auxCompStr << e.what() ;  \
		auxCompStr << mrpt::format( stuff, param1 ) << std::endl; \
		throw std::logic_error( auxCompStr.str() );\
	}\

/** For use in CSerializable implementations */
#define MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(__V) THROW_EXCEPTION(mrpt::format("Cannot parse object: unknown serialization version number: '%i'",static_cast<int>(__V)))


#if MRPT_HAS_ASSERT
	/** Defines an assertion mechanism.
	 * \note Do NOT put code that must be always executed inside this statement, but just comparisons. This is because users might require ASSERT_'s to be ignored for optimized releases.
	 * \sa MRPT_TRY_START, MRPT_TRY_END
	 */
#	define ASSERTMSG_(f,__ERROR_MSG) \
	{ \
	if (!(f)) THROW_EXCEPTION( ::std::string( __ERROR_MSG ) ); \
	}

	/** Defines an assertion mechanism.
	 * \note Do NOT put code that must be always executed inside this statement, but just comparisons. This is because users might require ASSERT_'s to be ignored for optimized releases.
	 * \sa MRPT_TRY_START, MRPT_TRY_END
	 */
#	define ASSERT_(f) \
		ASSERTMSG_(f, std::string("Assert condition failed: ") + ::std::string(#f) )

/** Throws an exception if the number is NaN, IND, or +/-INF, or return the same number otherwise.
  */
#define MRPT_CHECK_NORMAL_NUMBER(v) \
	{ \
		if (math::isNaN(v)) THROW_EXCEPTION("Check failed (value is NaN)"); \
		if (!math::isFinite(v)) THROW_EXCEPTION("Check failed (value is not finite)"); \
	}

// Static asserts: use compiler version if we have a modern GCC (>=4.3) or MSVC (>=2010) version, otherwise rely on custom implementation:
#if MRPT_CHECK_VISUALC_VERSION(10) || __has_extension(cxx_static_assert) || (MRPT_CHECK_GCC_VERSION(4,3) && MRPT_HAS_CXX11)
	#define MRPT_COMPILE_TIME_ASSERT(expression) static_assert(expression,#expression);
#else
	// The following macro is based on dclib:
	// Copyright (C) 2003  Davis E. King (davisking@users.sourceforge.net)
	// License: Boost Software License   See LICENSE.txt for the full license.
	namespace mrpt
	{
		namespace utils
		{
			template <bool value> struct compile_time_assert;
			template <> struct compile_time_assert<true> { enum {value=1};  };
		}
	}
	#define MRPT_COMPILE_TIME_ASSERT(expression) \
			typedef char BOOST_JOIN(MRPT_CTA, __LINE__)[::mrpt::utils::compile_time_assert<(bool)(expression)>::value]; extern BOOST_JOIN(MRPT_CTA, __LINE__) BOOST_JOIN(MRPT_DUMMYVAR_CTA, __LINE__);

#endif

	/** Assert comparing two values, reporting their actual values upon failure */
	#define ASSERT_EQUAL_( __A, __B)      { if (__A!=__B) { std::ostringstream __s__;__s__<<"ASSERT_EQUAL_("<<#__A<<","<<#__B<<") failed with\n"<<#__A<<"=" <<__A <<"\n"<<#__B<<"="<<__B; THROW_EXCEPTION(__s__.str()) } }
	#define ASSERT_NOT_EQUAL_( __A, __B)  { if (__A==__B) { std::ostringstream __s__;__s__<<"ASSERT_NOT_EQUAL_("<<#__A<<","<<#__B<<") failed with\n"<<#__A<<"=" <<__A <<"\n"<<#__B<<"="<<__B; THROW_EXCEPTION(__s__.str()) } }
	#define ASSERT_BELOW_( __A, __B)  { if (__A>=__B) { std::ostringstream __s__;__s__<<"ASSERT_BELOW_("<<#__A<<","<<#__B<<") failed with\n"<<#__A<<"=" <<__A <<"\n"<<#__B<<"="<<__B; THROW_EXCEPTION(__s__.str()) } }
	#define ASSERT_ABOVE_( __A, __B)  { if (__A<=__B) { std::ostringstream __s__;__s__<<"ASSERT_ABOVE_("<<#__A<<","<<#__B<<") failed with\n"<<#__A<<"=" <<__A <<"\n"<<#__B<<"="<<__B; THROW_EXCEPTION(__s__.str()) } }
	#define ASSERT_BELOWEQ_( __A, __B)  { if (__A>__B) { std::ostringstream __s__;__s__<<"ASSERT_BELOWEQ_("<<#__A<<","<<#__B<<") failed with\n"<<#__A<<"=" <<__A <<"\n"<<#__B<<"="<<__B; THROW_EXCEPTION(__s__.str()) } }
	#define ASSERT_ABOVEEQ_( __A, __B)  { if (__A<__B) { std::ostringstream __s__;__s__<<"ASSERT_ABOVEEQ_("<<#__A<<","<<#__B<<") failed with\n"<<#__A<<"=" <<__A <<"\n"<<#__B<<"="<<__B; THROW_EXCEPTION(__s__.str()) } }

	#define ASSERT_FILE_EXISTS_(FIL)      ASSERTMSG_( mrpt::system::fileExists(FIL), std::string("Assert file existence failed: ") + ::std::string(FIL) )
	#define ASSERT_DIRECTORY_EXISTS_(DIR) ASSERTMSG_( mrpt::system::directoryExists(DIR), std::string("Assert directory existence failed: ") + ::std::string(DIR) )

#else // MRPT_HAS_ASSERT
#	define ASSERTMSG_(f,__ERROR_MSG)  { }
#	define ASSERT_(f) { }
#	define MRPT_CHECK_NORMAL_NUMBER(val) { }
#	define MRPT_COMPILE_TIME_ASSERT(f) { }
#	define ASSERT_EQUAL_( __A, __B) { }
#	define ASSERT_NOT_EQUAL_( __A, __B) { }
#	define ASSERT_BELOW_( __A, __B)  { }
#	define ASSERT_ABOVE_( __A, __B)  { }
#	define ASSERT_BELOWEQ_( __A, __B)  { }
#	define ASSERT_ABOVEEQ_( __A, __B)  { }

#	define ASSERT_FILE_EXISTS_(FIL)      { }
#	define ASSERT_DIRECTORY_EXISTS_(DIR) { }
#endif // MRPT_HAS_ASSERT

/** Defines an assertion mechanism - only when compiled in debug.
 * \note Do NOT put code that must be always executed inside this statement, but just comparisons. This is because users might require ASSERT_'s to be ignored for optimized releases.
 * \sa MRPT_TRY_START, MRPT_TRY_END
 */
#ifdef _DEBUG
#	define ASSERTDEB_(f) ASSERT_(f)
#	define ASSERTDEBMSG_(f,__ERROR_MSG) ASSERTMSG_(f,__ERROR_MSG)
#else
#	define ASSERTDEB_(f) { }
#	define ASSERTDEBMSG_(f,__ERROR_MSG) { }
#endif


/** Can be used to avoid "not used parameters" warnings from the compiler
 */
#define MRPT_UNUSED_PARAM(a)		(void)(a)

#if MRPT_HAS_STACKED_EXCEPTIONS
	/** The start of a standard MRPT "try...catch()" block that allows tracing throw the call stack after an exception.
	  * \sa MRPT_TRY_END,MRPT_TRY_END_WITH_CLEAN_UP
	  */
#	define MRPT_TRY_START	\
	try { \

	/** The end of a standard MRPT "try...catch()" block that allows tracing throw the call stack after an exception.
	  * \sa MRPT_TRY_START,MRPT_TRY_END_WITH_CLEAN_UP
	  */
#	define MRPT_TRY_END	\
	} \
	catch (std::bad_alloc &) \
	{ throw; } \
	catch (std::exception &e) \
	{ \
		THROW_STACKED_EXCEPTION(e); \
	} \
	catch (...) \
	{ \
		THROW_EXCEPTION("Unexpected runtime error!"); \
	} \

	/** The end of a standard MRPT "try...catch()" block that allows tracing throw the call stack after an exception, including a "clean up" piece of code to be run before throwing the exceptions.
	  * \sa MRPT_TRY_END,MRPT_TRY_START
	  */
#	define MRPT_TRY_END_WITH_CLEAN_UP(stuff)	\
	} \
	catch (std::bad_alloc &) \
	{ throw; } \
	catch (std::exception &e) \
	{ \
		{stuff} \
		THROW_STACKED_EXCEPTION(e); \
	} \
	catch (...) \
	{ \
		{ stuff } \
		THROW_EXCEPTION("Unexpected runtime error!"); \
	} \

#else
#	define MRPT_TRY_START
#	define MRPT_TRY_END
#	define MRPT_TRY_END_WITH_CLEAN_UP(stuff)
#endif

#if MRPT_ENABLE_EMBEDDED_GLOBAL_PROFILER
#	define	MRPT_PROFILE_FUNC_START  ::mrpt::utils::CProfilerProxy BOOST_JOIN(__dum_var,__LINE__)( __CURRENT_FUNCTION_NAME__);
#else
#	define	MRPT_PROFILE_FUNC_START
#endif

// General macros for use within each MRPT method/function. They provide:
//  - Nested exception handling
//  - Automatic profiling stats (in Debug only)
// ---------------------------------------------------------
#define MRPT_START  \
	MRPT_PROFILE_FUNC_START \
	MRPT_TRY_START

#define MRPT_END  \
	MRPT_TRY_END

#define MRPT_END_WITH_CLEAN_UP(stuff) \
	MRPT_TRY_END_WITH_CLEAN_UP(stuff)

// Generic constants and defines:
// ---------------------------------------------------------
// M_PI: Rely on standard <cmath>
#ifndef M_2PI
#	define M_2PI 6.283185307179586476925286766559	// The 2*PI constant
#endif

#define M_PIf  3.14159265358979f
#define M_2PIf 6.28318530717959f

#if defined(HAVE_LONG_DOUBLE) && !defined(M_PIl)
#	define M_PIl 3.14159265358979323846264338327950288L
#	define M_2PIl (2.0L*3.14159265358979323846264338327950288L)
#endif


// Define a decl. modifier for printf-like format checks at compile time:
#ifdef __GNUC__
#	define MRPT_printf_format_check(_FMT_,_VARARGS_)  __attribute__ ((__format__ (__printf__, _FMT_,_VARARGS_)))
#else
#	define MRPT_printf_format_check(_FMT_,_VARARGS_)
#endif

// Define a decl. modifier for scanf-like format checks at compile time:
#ifdef __GNUC__
#	define MRPT_scanf_format_check(_FMT_,_VARARGS_)  __attribute__ ((__format__ (__scanf__, _FMT_,_VARARGS_)))
#else
#	define MRPT_scanf_format_check(_FMT_,_VARARGS_)
#endif


/** Tells the compiler we really want to inline that function */
#if (defined _MSC_VER) || (defined __INTEL_COMPILER)
#define MRPT_FORCE_INLINE __forceinline
#else
#define MRPT_FORCE_INLINE inline
#endif

/** Determines whether this is an X86 or AMD64 platform */
#if defined(__amd64__) || defined(__amd64) || defined(__x86_64__) || defined(__x86_64) || defined(_M_AMD64) || defined (_M_X64) \
	|| defined (__i386__)|| defined (__i386) || defined (_M_I86) || defined (i386) || defined(_M_IX86) || defined (_X86_)
#	define MRPT_IS_X86_AMD64  1
#endif

namespace mrpt
{
	// Redeclared here for convenience:
	std::string BASE_IMPEXP format(const char *fmt, ...) MRPT_printf_format_check(1,2);
}

#endif
