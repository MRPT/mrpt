/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#pragma once

#include <string>  // std::string, to_string()
#include <stdexcept>  // logic_error
#include <mrpt/core/common.h>
#include <mrpt/core/format.h>

   /** \def THROW_TYPED_EXCEPTION(msg,exceptionClass) */
#define THROW_TYPED_EXCEPTION(msg, exceptionClass)                             \
	do                                                                         \
	{                                                                          \
		std::string s = "\n\n =============== MRPT EXCEPTION =============\n"; \
		s += __CURRENT_FUNCTION_NAME__;                                        \
		s += ", line ";                                                        \
		s += mrpt::to_string(__LINE__);                                         \
		s += ":\n";                                                            \
		s += msg;                                                              \
		s += ":\n";                                                            \
		throw exceptionClass(s);                                               \
	} while (0)

/** \def THROW_EXCEPTION(msg);
* \param msg This can be a char*, a std::string, or a literal string.
* Defines a unified way of reporting exceptions
* \sa MRPT_TRY_START, MRPT_TRY_END, THROW_EXCEPTION_FMT
*/
#define THROW_EXCEPTION(msg) THROW_TYPED_EXCEPTION(msg, std::logic_error);

#define THROW_EXCEPTION_FMT(_FORMAT_STRING, ...) \
	THROW_EXCEPTION(mrpt::format(_FORMAT_STRING, __VA_ARGS__));

#define THROW_TYPED_EXCEPTION_FMT(exceptionClass, _FORMAT_STRING, ...) \
	THROW_TYPED_EXCEPTION(                                             \
		mrpt::format(_FORMAT_STRING, __VA_ARGS__), exceptionClass)

/** \def THROW_STACKED_EXCEPTION
 * \sa MRPT_TRY_START, MRPT_TRY_END
 */
#define THROW_STACKED_EXCEPTION(e)      \
	do                                  \
	{                                   \
		std::string s = e.what();       \
		s += "\n";                      \
		s += __FILE__;                  \
		s += ":";                       \
		s += mrpt::to_string(__LINE__);  \
		s += ": In `";                  \
		s += __CURRENT_FUNCTION_NAME__; \
		s += "`\n";                     \
		throw std::logic_error(s);      \
	} while (0)

/** \def THROW_STACKED_EXCEPTION_CUSTOM_MSG
  * \param e The caught exception.
  *	\param stuff Is a printf-like sequence of params, e.g: "The error happens
  *for x=%i",x
  */
#define THROW_STACKED_EXCEPTION_CUSTOM_MSG2(e, stuff, param1) \
	do                                                        \
	{                                                         \
		std::string s = e.what();                             \
		s += mrpt::format(stuff, param1);                     \
		s += "\n";                                            \
		throw std::logic_error(s);                            \
	} while (0)

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
	do                                                         \
	{                                                          \
		if (!(f)) THROW_EXCEPTION(::std::string(__ERROR_MSG)); \
	} while (0)

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
	do                              \
	{                               \
		ASSERT_(std::isfinite(v));  \
		ASSERT_(!std::isnan(v));    \
	} while (0)

// Static asserts: use compiler version if we have a modern GCC (>=4.3) or MSVC
// (>=2010) version, otherwise rely on custom implementation:
#define MRPT_COMPILE_TIME_ASSERT(expression) \
	static_assert(expression, #expression)

#define ASRT_FAIL(__CONDITIONSTR, __A, __B, __ASTR, __BSTR) \
	{                                                       \
		std::string s = __CONDITIONSTR;                     \
		s += "(";                                           \
		s += __ASTR;                                        \
		s += ",";                                           \
		s += __BSTR;                                        \
		s += ") failed with\n";                             \
		s += __ASTR;                                        \
		s += "=";                                           \
		s += mrpt::to_string(__A);                           \
		s += "\n";                                          \
		s += __BSTR;                                        \
		s += "=";                                           \
		s += mrpt::to_string(__B);                           \
		s += "\n";                                          \
		THROW_EXCEPTION(s);                                 \
	}

/** Assert comparing two values, reporting their actual values upon failure */
#define ASSERT_EQUAL_(__A, __B)                                          \
	do                                                                   \
	{                                                                    \
		if (__A != __B) ASRT_FAIL("ASSERT_EQUAL_", __A, __B, #__A, #__B) \
	} while (0)

#define ASSERT_NOT_EQUAL_(__A, __B)                                          \
	do                                                                       \
	{                                                                        \
		if (__A == __B) ASRT_FAIL("ASSERT_NOT_EQUAL_", __A, __B, #__A, #__B) \
	} while (0)

#define ASSERT_BELOW_(__A, __B)                                          \
	do                                                                   \
	{                                                                    \
		if (__A >= __B) ASRT_FAIL("ASSERT_BELOW_", __A, __B, #__A, #__B) \
	} while (0)

#define ASSERT_ABOVE_(__A, __B)                                          \
	do                                                                   \
	{                                                                    \
		if (__A <= __B) ASRT_FAIL("ASSERT_ABOVE_", __A, __B, #__A, #__B) \
	} while (0)

#define ASSERT_BELOWEQ_(__A, __B)                                         \
	do                                                                    \
	{                                                                     \
		if (__A > __B) ASRT_FAIL("ASSERT_BELOWEQ_", __A, __B, #__A, #__B) \
	} while (0)

#define ASSERT_ABOVEEQ_(__A, __B)                                         \
	do                                                                    \
	{                                                                     \
		if (__A < __B) ASRT_FAIL("ASSERT_ABOVEEQ_", __A, __B, #__A, #__B) \
	} while (0)

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
