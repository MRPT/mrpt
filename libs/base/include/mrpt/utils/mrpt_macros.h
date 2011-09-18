/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

#ifndef UTILSDEFS_H
#error "This file is intended for include from utils_defs.h only!"
#endif

#ifndef MRPT_MACROS_H
#define MRPT_MACROS_H

// A cross-compiler definition for "deprecated"-warnings
#if defined(__GNUC__) && (__GNUC__ - 0 > 3 || (__GNUC__ - 0 == 3 && __GNUC_MINOR__ - 0 >= 2))
   /* gcc >= 3.2 */
#   define MRPT_DEPRECATED_PRE(_MSG)
	// The "message" is not supported yet in GCC (JL: wait for gcc 4.4??)
	//#   define MRPT_DEPRECATED_POST(_MSG) __attribute__ ((deprecated(_MSG)))
#   define MRPT_DEPRECATED_POST(_MSG) __attribute__ ((deprecated))
# elif defined(_MSC_VER) && (_MSC_VER >= 1300)
  /* msvc >= 7 */
#   define MRPT_DEPRECATED_PRE(_MSG)  __declspec(deprecated (_MSG))
#   define MRPT_DEPRECATED_POST(_MSG)
# else
#  define MRPT_DEPRECATED_PRE(_MSG)
#  define MRPT_DEPRECATED_POST(_MSG)
# endif

/** Usage: MRPT_DECLARE_DEPRECATED_FUNCTION("Use XX instead", void myFunc(double)); */
#define MRPT_DECLARE_DEPRECATED_FUNCTION(__MSG, __FUNC) MRPT_DEPRECATED_PRE(__MSG) __FUNC MRPT_DEPRECATED_POST(__MSG)

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

#define MRPT_WARNING(x) MRPT_MSG_PRAGMA("Warning: " #x)
#define MRPT_TODO(x)	MRPT_MSG_PRAGMA("TODO: " #x)

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

/** Used after member declarations */
#define MRPT_NO_THROWS		throw()


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
#if defined(__BORLANDC__)
		#define	__CURRENT_FUNCTION_NAME__	__FUNC__
#elif defined(_MSC_VER) && (_MSC_VER>=1300)
		#define	__CURRENT_FUNCTION_NAME__	__FUNCTION__
#elif defined(_MSC_VER) && (_MSC_VER<1300)
		// Visual C++ 6 HAS NOT A __FUNCTION__ equivalent.
#define	__CURRENT_FUNCTION_NAME__	::system::extractFileName(__FILE__).c_str()
#else
		#define	__CURRENT_FUNCTION_NAME__	__PRETTY_FUNCTION__
#endif


/** \def MRPT_DEBUG_BREAKPOINT(_msg)  
  *  Only if compiled in debug (_DEBUG defined), calls mrpt::system::breakpoint() with the given message. 
  *  All MRPT exceptions use this macro, so mrpt::system::breakpoint() is the ideal point to set a breakpoint 
  *  and catch exception before they're actually raised.
  */
#ifdef _DEBUG
#	define MRPT_DEBUG_BREAKPOINT(_msg)  { mrpt::system::breakpoint(std::string(_msg)); }
#else
#	define MRPT_DEBUG_BREAKPOINT(_msg)  { }
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
		auxCompStr << mrpt::system::stack_trace(); \
		MRPT_DEBUG_BREAKPOINT(msg) \
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
		auxCompStr << mrpt::system::stack_trace(); \
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
		auxCompStr << mrpt::system::stack_trace(); \
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
		auxCompStr << mrpt::system::stack_trace(); \
		throw exceptionClass( auxCompStr.str() );\
	}\


/** \def THROW_STACKED_EXCEPTION
 * \sa MRPT_TRY_START, MRPT_TRY_END
 */
#define THROW_STACKED_EXCEPTION(e)	\
	{\
		std::string str( e.what() );\
		if (str.find("MRPT stack trace")==std::string::npos) \
		{ \
			str+= __CURRENT_FUNCTION_NAME__;\
			str+= mrpt::format(", line %i:\n", __LINE__ );\
			if (str.size()>3000) { std::cerr << "TOO MANY STACKED EXCEPTIONS!: " << std::endl << str << std::endl; abort(); } \
			throw std::logic_error( str );\
		} \
		else throw std::logic_error( e.what() );\
	}\

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
#define MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(V) THROW_EXCEPTION(mrpt::format("Cannot parse object: unknown serialization version number: '%i'",static_cast<int>(version)))


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
			typedef char BOOST_JOIN(MRPT_CTA, __LINE__)[::mrpt::utils::compile_time_assert<(bool)(expression)>::value];

	/** Assert comparing two values, reporting their actual values upon failure */
	#define ASSERT_EQUAL_( __A, __B)      { if (__A!=__B) { std::ostringstream s;s<<"ASSERT_EQUAL_("<<#__A<<","<<#__B<<") failed with\n"<<#__A<<"=" <<__A <<"\n"<<#__B<<"="<<__B; THROW_EXCEPTION(s.str()) } }
	#define ASSERT_NOT_EQUAL_( __A, __B)  { if (__A==__B) { std::ostringstream s;s<<"ASSERT_NOT_EQUAL_("<<#__A<<","<<#__B<<") failed with\n"<<#__A<<"=" <<__A <<"\n"<<#__B<<"="<<__B; THROW_EXCEPTION(s.str()) } }
	#define ASSERT_BELOW_( __A, __B)  { if (__A>=__B) { std::ostringstream s;s<<"ASSERT_BELOW_("<<#__A<<","<<#__B<<") failed with\n"<<#__A<<"=" <<__A <<"\n"<<#__B<<"="<<__B; THROW_EXCEPTION(s.str()) } }
	#define ASSERT_ABOVE_( __A, __B)  { if (__A<=__B) { std::ostringstream s;s<<"ASSERT_ABOVE_("<<#__A<<","<<#__B<<") failed with\n"<<#__A<<"=" <<__A <<"\n"<<#__B<<"="<<__B; THROW_EXCEPTION(s.str()) } }
	#define ASSERT_BELOWEQ_( __A, __B)  { if (__A>__B) { std::ostringstream s;s<<"ASSERT_BELOWEQ_("<<#__A<<","<<#__B<<") failed with\n"<<#__A<<"=" <<__A <<"\n"<<#__B<<"="<<__B; THROW_EXCEPTION(s.str()) } }
	#define ASSERT_ABOVEEQ_( __A, __B)  { if (__A<__B) { std::ostringstream s;s<<"ASSERT_ABOVEEQ_("<<#__A<<","<<#__B<<") failed with\n"<<#__A<<"=" <<__A <<"\n"<<#__B<<"="<<__B; THROW_EXCEPTION(s.str()) } }

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
#ifndef M_PI
#	define M_PI 3.14159265358979323846264338327950288		// PI constant
#endif

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


/** Used after member declarations */
#define MRPT_NO_THROWS		throw()

/** Tells the compiler we really want to inline that function */
#if (defined _MSC_VER) || (defined __INTEL_COMPILER)
#define MRPT_FORCE_INLINE __forceinline
#else
#define MRPT_FORCE_INLINE inline
#endif

#endif
