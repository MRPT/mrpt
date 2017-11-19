/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#pragma once

#include <sstream>  // ostringstream
#include <stdexcept>  // logic_error
#include <mrpt/macros/common.h>
#include <mrpt/macros/format.h>

/** \def THROW_EXCEPTION(msg)
 * \param msg This can be a char*, a std::string, or a literal string.
 * Defines a unified way of reporting exceptions
 * \sa MRPT_TRY_START, MRPT_TRY_END, THROW_EXCEPTION_FMT
 */
#define THROW_EXCEPTION(msg)                                                 \
	{                                                                        \
		std::ostringstream auxCompStr;                                       \
		auxCompStr << "\n\n =============== MRPT EXCEPTION =============\n"; \
		auxCompStr << __CURRENT_FUNCTION_NAME__ << ", line " << __LINE__     \
				   << ":\n";                                                 \
		auxCompStr << msg << std::endl;                                      \
		throw std::logic_error(auxCompStr.str());                            \
	}

#define THROW_EXCEPTION_FMT(_FORMAT_STRING, ...) \
	THROW_EXCEPTION(mrpt::format(_FORMAT_STRING, __VA_ARGS__))

/** \def THROW_TYPED_EXCEPTION(msg,exceptionClass)
 * Defines a unified way of reporting exceptions of type different than
 * "std::exception"
 * \sa MRPT_TRY_START, MRPT_TRY_END
 */
#define THROW_TYPED_EXCEPTION(msg, exceptionClass)                           \
	{                                                                        \
		std::ostringstream auxCompStr;                                       \
		auxCompStr << "\n\n =============== MRPT EXCEPTION =============\n"; \
		auxCompStr << __CURRENT_FUNCTION_NAME__ << ", line " << __LINE__     \
				   << ":\n";                                                 \
		auxCompStr << msg << std::endl;                                      \
		throw exceptionClass(auxCompStr.str());                              \
	}

#define THROW_TYPED_EXCEPTION_FMT(exceptionClass, _FORMAT_STRING, ...) \
	THROW_TYPED_EXCEPTION(                                             \
		mrpt::format(_FORMAT_STRING, __VA_ARGS__), exceptionClass)

/** \def THROW_STACKED_EXCEPTION
 * \sa MRPT_TRY_START, MRPT_TRY_END
 */
#define THROW_STACKED_EXCEPTION(e)                            \
	{                                                         \
		std::string _tse_str(mrpt::format("%s\n", e.what())); \
		_tse_str += mrpt::format(                             \
			" %s:%i: In `%s`\n", __FILE__, __LINE__,          \
			__CURRENT_FUNCTION_NAME__);                       \
		throw std::logic_error(_tse_str);                     \
	}

/** \def THROW_STACKED_EXCEPTION_CUSTOM_MSG
  * \param e The caught exception.
  *	\param msg Is a char* or std::string.
  */
#define THROW_STACKED_EXCEPTION_CUSTOM_MSG1(e, msg) \
	{                                               \
		std::ostringstream auxCompStr;              \
		auxCompStr << e.what();                     \
		auxCompStr << msg << std::endl;             \
		throw std::logic_error(auxCompStr.str());   \
	}

/** \def THROW_STACKED_EXCEPTION_CUSTOM_MSG
  * \param e The caught exception.
  *	\param stuff Is a printf-like sequence of params, e.g: "The error happens
  *for x=%i",x
  */
#define THROW_STACKED_EXCEPTION_CUSTOM_MSG2(e, stuff, param1)   \
	{                                                           \
		std::ostringstream auxCompStr;                          \
		auxCompStr << e.what();                                 \
		auxCompStr << mrpt::format(stuff, param1) << std::endl; \
		throw std::logic_error(auxCompStr.str());               \
	}

/** For use in CSerializable implementations */
#define MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(__V)                          \
	THROW_EXCEPTION(                                                           \
		mrpt::format(                                                          \
			"Cannot parse object: unknown serialization version number: '%i'", \
			static_cast<int>(__V)))

/** Defines an assertion mechanism.
 * \note Do NOT put code that must be always executed inside this statement, but
 * just comparisons. This is because users might require ASSERT_'s to be ignored
 * for optimized releases.
 * \sa MRPT_TRY_START, MRPT_TRY_END
 */
#define ASSERTMSG_(f, __ERROR_MSG)                             \
	{                                                          \
		if (!(f)) THROW_EXCEPTION(::std::string(__ERROR_MSG)); \
	}

/** Defines an assertion mechanism.
 * \note Do NOT put code that must be always executed inside this statement, but
 * just comparisons. This is because users might require ASSERT_'s to be ignored
 * for optimized releases.
 * \sa MRPT_TRY_START, MRPT_TRY_END
 */
#define ASSERT_(f) \
	ASSERTMSG_(f, std::string("Assert condition failed: ") + ::std::string(#f))

/** Throws an exception if the number is NaN, IND, or +/-INF, or return the same
 * number otherwise. */
#define MRPT_CHECK_NORMAL_NUMBER(v) \
	{                               \
		ASSERT_(std::isfinite(v));  \
		ASSERT_(!std::isnan(v));    \
	}

// Static asserts: use compiler version if we have a modern GCC (>=4.3) or MSVC
// (>=2010) version, otherwise rely on custom implementation:
#define MRPT_COMPILE_TIME_ASSERT(expression) \
	static_assert(expression, #expression);

/** Assert comparing two values, reporting their actual values upon failure */
#define ASSERT_EQUAL_(__A, __B)                              \
	{                                                        \
		if (__A != __B)                                      \
		{                                                    \
			std::ostringstream __s__;                        \
			__s__ << "ASSERT_EQUAL_(" << #__A << "," << #__B \
				  << ") failed with\n"                       \
				  << #__A << "=" << __A << "\n"              \
				  << #__B << "=" << __B;                     \
			THROW_EXCEPTION(__s__.str())                     \
		}                                                    \
	}
#define ASSERT_NOT_EQUAL_(__A, __B)                              \
	{                                                            \
		if (__A == __B)                                          \
		{                                                        \
			std::ostringstream __s__;                            \
			__s__ << "ASSERT_NOT_EQUAL_(" << #__A << "," << #__B \
				  << ") failed with\n"                           \
				  << #__A << "=" << __A << "\n"                  \
				  << #__B << "=" << __B;                         \
			THROW_EXCEPTION(__s__.str())                         \
		}                                                        \
	}
#define ASSERT_BELOW_(__A, __B)                              \
	{                                                        \
		if (__A >= __B)                                      \
		{                                                    \
			std::ostringstream __s__;                        \
			__s__ << "ASSERT_BELOW_(" << #__A << "," << #__B \
				  << ") failed with\n"                       \
				  << #__A << "=" << __A << "\n"              \
				  << #__B << "=" << __B;                     \
			THROW_EXCEPTION(__s__.str())                     \
		}                                                    \
	}
#define ASSERT_ABOVE_(__A, __B)                              \
	{                                                        \
		if (__A <= __B)                                      \
		{                                                    \
			std::ostringstream __s__;                        \
			__s__ << "ASSERT_ABOVE_(" << #__A << "," << #__B \
				  << ") failed with\n"                       \
				  << #__A << "=" << __A << "\n"              \
				  << #__B << "=" << __B;                     \
			THROW_EXCEPTION(__s__.str())                     \
		}                                                    \
	}
#define ASSERT_BELOWEQ_(__A, __B)                              \
	{                                                          \
		if (__A > __B)                                         \
		{                                                      \
			std::ostringstream __s__;                          \
			__s__ << "ASSERT_BELOWEQ_(" << #__A << "," << #__B \
				  << ") failed with\n"                         \
				  << #__A << "=" << __A << "\n"                \
				  << #__B << "=" << __B;                       \
			THROW_EXCEPTION(__s__.str())                       \
		}                                                      \
	}
#define ASSERT_ABOVEEQ_(__A, __B)                              \
	{                                                          \
		if (__A < __B)                                         \
		{                                                      \
			std::ostringstream __s__;                          \
			__s__ << "ASSERT_ABOVEEQ_(" << #__A << "," << #__B \
				  << ") failed with\n"                         \
				  << #__A << "=" << __A << "\n"                \
				  << #__B << "=" << __B;                       \
			THROW_EXCEPTION(__s__.str())                       \
		}                                                      \
	}

#define ASSERT_FILE_EXISTS_(FIL)       \
	ASSERTMSG_(                        \
		mrpt::system::fileExists(FIL), \
		std::string("Assert file existence failed: ") + ::std::string(FIL))
#define ASSERT_DIRECTORY_EXISTS_(DIR)                        \
	ASSERTMSG_(                                              \
		mrpt::system::directoryExists(DIR),                  \
		std::string("Assert directory existence failed: ") + \
			::std::string(DIR))


/** Defines an assertion mechanism - only when compiled in debug.
 * \note Do NOT put code that must be always executed inside this statement, but
 * just comparisons. This is because users might require ASSERT_'s to be ignored
 * for optimized releases.
 * \sa MRPT_TRY_START, MRPT_TRY_END
 */
#ifdef _DEBUG
#define ASSERTDEB_(f) ASSERT_(f)
#define ASSERTDEBMSG_(f, __ERROR_MSG) ASSERTMSG_(f, __ERROR_MSG)
#else
#define ASSERTDEB_(f) \
	{                 \
	}
#define ASSERTDEBMSG_(f, __ERROR_MSG) \
	{                                 \
	}
#endif

/** The start of a standard MRPT "try...catch()" block that allows tracing throw
 * the call stack after an exception.
  * \sa MRPT_TRY_END,MRPT_TRY_END_WITH_CLEAN_UP
  */
#define MRPT_TRY_START \
	try                \
	{
/** The end of a standard MRPT "try...catch()" block that allows tracing throw
 * the call stack after an exception.
  * \sa MRPT_TRY_START,MRPT_TRY_END_WITH_CLEAN_UP
  */
#define MRPT_TRY_END                                           \
	}                                                          \
	catch (std::bad_alloc&) { throw; }                         \
	catch (std::exception & e) { THROW_STACKED_EXCEPTION(e); } \
	catch (...) { THROW_EXCEPTION("Unexpected runtime error!"); }
/** The end of a standard MRPT "try...catch()" block that allows tracing throw
 * the call stack after an exception, including a "clean up" piece of code to be
 * run before throwing the exceptions.
  * \sa MRPT_TRY_END,MRPT_TRY_START
  */
#define MRPT_TRY_END_WITH_CLEAN_UP(stuff)             \
	}                                                 \
	catch (std::bad_alloc&) { throw; }                \
	catch (std::exception & e)                        \
	{                                                 \
		{                                             \
			stuff                                     \
		}                                             \
		THROW_STACKED_EXCEPTION(e);                   \
	}                                                 \
	catch (...)                                       \
	{                                                 \
		{                                             \
			stuff                                     \
		}                                             \
		THROW_EXCEPTION("Unexpected runtime error!"); \
	}


#if MRPT_ENABLE_EMBEDDED_GLOBAL_PROFILER
#define MRPT_PROFILE_FUNC_START               \
	::mrpt::utils::CProfilerProxy BOOST_JOIN( \
		__dum_var, __LINE__)(__CURRENT_FUNCTION_NAME__);
#else
#define MRPT_PROFILE_FUNC_START
#endif

// General macros for use within each MRPT method/function. They provide:
//  - Nested exception handling
//  - Automatic profiling stats (in Debug only)
// ---------------------------------------------------------
#define MRPT_START          \
	MRPT_PROFILE_FUNC_START \
	MRPT_TRY_START

#define MRPT_END MRPT_TRY_END

#define MRPT_END_WITH_CLEAN_UP(stuff) MRPT_TRY_END_WITH_CLEAN_UP(stuff)

