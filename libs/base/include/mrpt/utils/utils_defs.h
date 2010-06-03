/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2010  University of Malaga                           |
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
#define UTILSDEFS_H

// This header will be included in ALL mrpt libs and programs.

/* ------------------------------------
          Disable some warnings
   ------------------------------------ */
#if defined(_MSC_VER)
	#pragma warning(disable:4786) // (Compiler: Visual C++) Disable warning for too long debug names:
	#pragma warning(disable:4702) // (Compiler: Visual C++) Disable warning for unreachable code (I don't know why some of these errors appear in the STANDARD LIBRARY headers with Visual Studio 2003!):
	#pragma warning(disable:4244) // (Compiler: Visual C++) Conversion double->float
	#pragma warning(disable:4305)
	#pragma warning(disable:4267)
	#pragma warning(disable:4290) // Visual C++ does not implement decl. specifiers: throw(A,B,...)
	#pragma warning(disable:4251) // Visual C++ 2003+ warnings on STL classes when exporting to DLL...
	#pragma warning(disable:4275)
	#if (_MSC_VER >= 1400 )
		// MS believes they have the right to deprecate functions in the C++ Standard STL... disable their warnings:
		#define _SCL_SECURE_NO_WARNINGS
		#pragma warning(disable:4996)
		// For the new secure library in VC++8
		#if !defined(_CRT_SECURE_CPP_OVERLOAD_STANDARD_NAMES)
			#define _CRT_SECURE_CPP_OVERLOAD_STANDARD_NAMES 1
		#endif
	#endif
#endif

#if defined(__BORLANDC__)
	#pragma warn -8027	// (Compiler: Borland C++) Disable a warning for inline functions
	#pragma warn -8012	// (Compiler: Borland C++) Disable a warning for override of virtual functions, while in reality there are many virtual function with different parameters
	#pragma warn -8022
#endif

#include <mrpt/config.h>
#include <mrpt/system/os.h>

#include <mrpt/utils/boost_join.h>

/* ------------------------------------------------
    Automatically tell the linker to include
	 the correct MRPT ".lib" file into the
	 user's application.
   ----------------------------------------------- */
#include <mrpt/base/link_pragmas.h>


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

// Enable leak memory debugging:
#if defined(_DEBUG) && defined(_MSC_VER) && (_MSC_VER>=1400)
	#define _CRTDBG_MAP_ALLOC
	#include <stdlib.h>
	#include <crtdbg.h>
#endif

// Utils C++ headers:
#include <cstddef>
#include <stdlib.h>
#include <cstdlib>
#include <cmath>

#if HAVE_ALLOCA_H
#include <alloca.h>
#endif

// C++ STD Library:
#include <vector>
#include <set>
#include <algorithm>
#include <iterator>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <cstring>
#include <exception>
#include <stdexcept>
#include <limits>
#include <sstream>

// STL+ library:
#include <mrpt/otherlibs/stlplus/smart_ptr.hpp>

// Standard elemental types:
#include <mrpt/utils/types.h>

/** This is the global namespace for all Mobile Robot Programming Toolkit (MRPT) libraries.
 */
namespace mrpt
{
	/** A std::string version of C sprintf.
	  *  You can call this to obtain a std::string using printf-like syntax.
	  *  Based on very nice code by Paul Senzee, published at http://senzee.blogspot.com/2006/05/c-formatting-stdstring.html
	  *  Function implemented in format.cpp
	  */
	std::string BASE_IMPEXP format(const char *fmt, ...) MRPT_printf_format_check(1,2);

	namespace utils
	{
		class CFileStream;
		void BASE_IMPEXP global_profiler_enter(const char *func_name) MRPT_NO_THROWS;
		void BASE_IMPEXP global_profiler_leave(const char *func_name) MRPT_NO_THROWS;

		struct CProfilerProxy {
			const char*f;
			CProfilerProxy(const char*func_name) : f(func_name) { global_profiler_enter(f); }
			~CProfilerProxy() { global_profiler_leave(f); }
		};
	}
	namespace system
	{
		// Forward definition: (Required for Visual C++ 6 implementation of THROW_EXCEPTION...)
		std::string  BASE_IMPEXP extractFileName(const std::string &filePath);
		std::string BASE_IMPEXP stack_trace(bool calling_from_exception);
	}

	namespace math
	{
	    bool BASE_IMPEXP isNaN(float v) MRPT_NO_THROWS;
	    bool BASE_IMPEXP isNaN(double v) MRPT_NO_THROWS;
	    bool BASE_IMPEXP isFinite(float v) MRPT_NO_THROWS;
	    bool BASE_IMPEXP isFinite(double v) MRPT_NO_THROWS;
	}
} // End of namespace


/** A macro for obtaining the name of the current function:
  */
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
		ASSERTMSG_(f, "Assert condition failed: " + ::std::string(#f) )

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

#else
#	define ASSERTMSG_(f,__ERROR_MSG)  { }
#	define ASSERT_(f) { }
#	define MRPT_CHECK_NORMAL_NUMBER(val) { }
#	define MRPT_COMPILE_TIME_ASSERT(f) { }
#endif

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
	catch (std::bad_alloc &e) \
	{ \
		std::cerr << "MRPT CRITICAL ERROR: Out of memory:\n" << e.what() << std::endl; \
		exit(-1); throw std::runtime_error("dummy"); \
	} \
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
	catch (std::bad_alloc &e) \
	{ \
		std::cerr << "MRPT CRITICAL ERROR: Out of memory:\n" << e.what() << std::endl; \
		exit(-1); throw std::runtime_error("dummy"); \
	} \
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


// Avoid conflicting declaration of max macro in windows headers
#if defined(MRPT_OS_WINDOWS) && !defined(NOMINMAX)
#	define NOMINMAX
#	ifdef max
#		undef	max
#		undef	min
#	endif
#endif

//  We want to avoid defining "max" & "min" as #define's since it create conflicts
//    with methods, variables, etc... with the same name in some compilers.
// Use std::max & std::min for all compilers by default, but for MSVC6 it does not exist:
#if defined(_MSC_VER) && (_MSC_VER<1300)
#	ifndef max
		namespace std
		{
			template<class T> inline const T max(const T& A,const T& B) { return A>B ? A:B; }
			template<class T> inline const T min(const T& A,const T& B) { return A<B ? A:B; }
		}
#	else
#		define  MAX3_MSVC6_VERSION
#	endif
#endif

// Min & Max:
#ifndef MAX3_MSVC6_VERSION
	template<typename T> inline const T  min3(const T& A, const T& B,const T& C) { return std::min<T>(A, std::min<T>(B,C) ); }
	template<typename T> inline const T  max3(const T& A, const T& B,const T& C) { return std::max<T>(A, std::max<T>(B,C) ); }
#else
#	define max3(A,B,C) max(A,max(B,C))
#	define min3(A,B,C) min(A,min(B,C))
#endif

namespace mrpt
{
	namespace math
	{
		// This inline function is used everywhere, so just move it here even it's not a forward declaration!
		/*! Returns the size of the matrix in the i'th dimension: 1=rows, 2=columns (MATLAB-compatible function)
		  *  \note Template argument MATRIXLIKE can be: CMatrixTemplate, CMatrixTemplateNumeric, CMatrixFixedNumeric
		  */
		template <class MATRIXLIKE>
		inline size_t size( const MATRIXLIKE& m, int dim )
		{
			if (dim==1)
				return m.getRowCount();
			else if (dim==2)
				return m.getColCount();
			else THROW_EXCEPTION_CUSTOM_MSG1("size: Queried matrix dimension must be 1 or 2. Called with i=%i",dim);
		}
	}


	namespace utils
	{
		/** Degrees to radians */
		inline double DEG2RAD(const double x) { return x*M_PI/180.0;	}

		/** Degrees to radians */
		inline float DEG2RAD(const float x) { return x*M_PIf/180.0f; }

		/** Degrees to radians */
		inline float DEG2RAD(const int x) { return x*M_PIf/180.0f; }

		/** Radians to degrees */
		inline double RAD2DEG(const double x) { return x*180.0/M_PI; }

		/** Radians to degrees */
		inline float RAD2DEG(const float x) { return x*180.0f/M_PIf; }

#	ifdef HAVE_LONG_DOUBLE
		/** Degrees to radians */
		inline long double DEG2RAD(const long double x) { return x*M_PIl/180.0;	}
		/** Radians to degrees */
		inline long double RAD2DEG(const long double x) { return x*180.0/M_PIl; }
#	endif

		/** Returns the sign of X as "1" or "-1" */
		template <typename T>
		inline int sign(T x) { return x<0 ? -1:1; }

		/** Returns the sign of X as "0", "1" or "-1" */
		template <typename T>
		inline int signWithZero(T x)	{ return x==0?0:sign(x);}

		/** Returns the closer integer (int) to x */
		template <typename T>
		inline int round(const T value)
		{
		#if MRPT_HAS_SSE2
			__m128d t = _mm_set_sd( value );
			return _mm_cvtsd_si32(t);
		#elif (defined WIN32 || defined _WIN32) && !defined WIN64 && !defined _WIN64 && defined _MSC_VER
			int t;
			__asm
			{
				fld value;
				fistp t;
			}
			return t;
		#elif defined HAVE_LRINT || defined __GNUC__
			return static_cast<int>(lrint(value));
		#else
			return static_cast<int>(value + 0.5);
		#endif
		}

		/** Returns the closer integer (long) to x */
		template <typename T>
		inline long round_long(const T value)
		{
		#if MRPT_HAS_SSE2 && MRPT_WORD_SIZE==64
			__m128d t = _mm_set_sd( value );
			return _mm_cvtsd_si64(t);
		#elif (defined WIN32 || defined _WIN32) && !defined WIN64 && !defined _WIN64 && defined _MSC_VER
			long t;
			__asm
			{
				fld value;
				fistp t;
			}
			return t;
		#elif defined HAVE_LRINT || defined __GNUC__
			return lrint(value);
		#else
			return static_cast<long>(value + 0.5);
		#endif
		}

		/** Rounds toward zero  */
		template <typename T>
		inline int fix(T x) { return  x>0 ? static_cast<int>(floor(static_cast<double>(x))) : static_cast<int>(ceil(static_cast<double>(x))) ; }

		/** Inline function for the square of a number. */
		template<class T>
		inline T square(const T& x)    { return x*x; }


		/** Utility to get a cast'ed pointer from a smart pointer */
		template <class R, class P>
		inline R* getAs(stlplus::smart_ptr_clone<P> &o) { return static_cast<R*>( & (*o) ); }

		/** Utility to get a cast'ed pointer from a smart pointer */
		template <class R, class P>
		inline const R* getAs(const stlplus::smart_ptr_clone<P> &o) { return static_cast<const R*>( & (*o) ); }


		/** Reverse the order of the bytes of a given type (useful for transforming btw little/big endian)  */
		template <class T> void reverseBytes(const T &v_in, T& v_out)
		{
			v_out = v_in;
			uint8_t *ptr = reinterpret_cast<uint8_t*>(&v_out);
			std::reverse(ptr,ptr+sizeof(T));
		}

		/** If the second argument is below the first one, set the first argument to this lower value. */
		template <typename T,typename K>
		inline void keep_min(T &var,  const K test_val) {
			if (test_val<var) var = test_val;
		}

		/** If the second argument is above the first one, set the first argument to this higher value. */
		template <typename T,typename K>
		inline void keep_max(T &var,  const K test_val) {
			if (test_val>var) var = test_val;
		}

		/** Calls "delete" to free an object only if the pointer is not NULL, then set the pointer to NULL. */
		template <class T>
		void delete_safe(T *& ptr) {
			if (ptr) {
				delete ptr;
				ptr = NULL;
			}
		}

	} // End of namespace
} // end of namespace
#endif
