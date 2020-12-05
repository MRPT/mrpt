/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/core/common.h>
#include <mrpt/core/format.h>
#include <stdexcept>  // logic_error
#include <string>  // std::string, to_string()
#include <string_view>

namespace mrpt::internal
{
std::string exception_line_msg(
	const std::string_view& msg, const char* filename, unsigned int line,
	const char* function_name);

template <typename A, typename B>
inline std::string asrt_fail(
	std::string s, A&& a, B&& b, const char* astr, const char* bstr)
{
	s += "(";
	s += astr;
	s += ",";
	s += bstr;
	s += ") failed with\n";
	s += astr;
	s += "=";
	s += mrpt::to_string(a);
	s += "\n";
	s += bstr;
	s += "=";
	s += mrpt::to_string(b);
	s += "\n";
	return s;
}
}  // namespace mrpt::internal

namespace mrpt
{
/** Builds a nice textual representation of a nested exception, which if
 * generated using MRPT macros (THROW_EXCEPTION,...) in between
 * MRPT_START/MRPT_END macros, will contain function names and line numbers
 * across the call stack at the original throw point.
 * See example of use in \ref mrpt_core_grp
 * Uses C++11 throw_with_nested(), rethrow_if_nested().
 * \ingroup mrpt_core_grp
 */
std::string exception_to_str(const std::exception& e);

/** \def THROW_TYPED_EXCEPTION(msg,exceptionClass) */
#define THROW_TYPED_EXCEPTION(msg, exceptionClass)           \
	throw exceptionClass(mrpt::internal::exception_line_msg( \
		msg, __FILE__, __LINE__, __CURRENT_FUNCTION_NAME__))

/** \def THROW_EXCEPTION(msg);
 * \param msg This can be a char*, a std::string, or a literal string.
 * Defines a unified way of reporting exceptions
 * \sa MRPT_TRY_START, MRPT_TRY_END, THROW_EXCEPTION_FMT
 */
#define THROW_EXCEPTION(msg) THROW_TYPED_EXCEPTION(msg, std::logic_error)

#define THROW_EXCEPTION_FMT(_FORMAT_STRING, ...) \
	THROW_EXCEPTION(mrpt::format(_FORMAT_STRING, __VA_ARGS__))

#define THROW_TYPED_EXCEPTION_FMT(exceptionClass, _FORMAT_STRING, ...) \
	THROW_TYPED_EXCEPTION(                                             \
		mrpt::format(_FORMAT_STRING, __VA_ARGS__), exceptionClass)

/** \def THROW_STACKED_EXCEPTION
 * \sa MRPT_TRY_START, MRPT_TRY_END
 */
#define THROW_STACKED_EXCEPTION                              \
	std::throw_with_nested(                                  \
		std::logic_error(mrpt::internal::exception_line_msg( \
			"Called from here.", __FILE__, __LINE__,         \
			__CURRENT_FUNCTION_NAME__)))

/** \def THROW_STACKED_EXCEPTION_CUSTOM_MSG
 * \param e The caught exception.
 *	\param stuff Is a printf-like sequence of params, e.g: "The error happens
 *for x=%i",x
 */
#define THROW_STACKED_EXCEPTION_CUSTOM_MSG2(stuff, param1)   \
	std::throw_with_nested(                                  \
		std::logic_error(mrpt::internal::exception_line_msg( \
			mrpt::format(stuff, param1), __FILE__, __LINE__, \
			__CURRENT_FUNCTION_NAME__)))

/** For use in CSerializable implementations */
#define MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(__V)                      \
	THROW_EXCEPTION(mrpt::format(                                          \
		"Cannot parse object: unknown serialization version number: '%i'", \
		static_cast<int>(__V)))

/** Throws a stacked exception if condition "f" is false; with custom message.
 * \sa MRPT_TRY_START, MRPT_TRY_END
 */
#define ASSERTMSG_(f, __ERROR_MSG)                             \
	do                                                         \
	{                                                          \
		if (!(f)) THROW_EXCEPTION(::std::string(__ERROR_MSG)); \
	} while (0)

/** Throws a stacked exception if condition "f" is false.
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

#define ASRT_FAIL(__CONDITIONSTR, __A, __B, __ASTR, __BSTR) \
	THROW_EXCEPTION(                                        \
		mrpt::internal::asrt_fail(__CONDITIONSTR, __A, __B, __ASTR, __BSTR));

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

/** Checks two float/double values, reporting their values upon failure */
#define ASSERT_NEAR_(__A, __B, __TOLERANCE)                 \
	do                                                      \
	{                                                       \
		const auto diff = std::abs(__A - __B);              \
		if (diff > __TOLERANCE)                             \
			ASRT_FAIL("ASSERT_NEAR_", __A, __B, #__A, #__B) \
	} while (0)

/** Checks A<B */
#define ASSERT_LT_(__A, __B)                                          \
	do                                                                \
	{                                                                 \
		if (__A >= __B) ASRT_FAIL("ASSERT_LT_", __A, __B, #__A, #__B) \
	} while (0)

/** Checks A<=B */
#define ASSERT_LE_(__A, __B)                                         \
	do                                                               \
	{                                                                \
		if (__A > __B) ASRT_FAIL("ASSERT_LE_", __A, __B, #__A, #__B) \
	} while (0)

/** Checks A>B */
#define ASSERT_GT_(__A, __B)                                          \
	do                                                                \
	{                                                                 \
		if (__A <= __B) ASRT_FAIL("ASSERT_GT_", __A, __B, #__A, #__B) \
	} while (0)

/** Checks A>=B */
#define ASSERT_GE_(__A, __B)                                         \
	do                                                               \
	{                                                                \
		if (__A < __B) ASRT_FAIL("ASSERT_GE_", __A, __B, #__A, #__B) \
	} while (0)

// ------- Deprecated ---------
#define ASSERT_BELOW_(__A, __B) ASSERT_LT_(__A, __B)
#define ASSERT_ABOVE_(__A, __B) ASSERT_GT_(__A, __B)
#define ASSERT_BELOWEQ_(__A, __B) ASSERT_LE_(__A, __B)
#define ASSERT_ABOVEEQ_(__A, __B) ASSERT_GE_(__A, __B)
// ------- End deprecated -----

#ifdef _DEBUG
#define ASSERTDEB_(f) ASSERT_(f)
#define ASSERTDEBMSG_(f, __ERROR_MSG) ASSERTMSG_(f, __ERROR_MSG)
#define ASSERTDEB_EQUAL_(__A, __B) ASSERT_EQUAL_(__A, __B)
#define ASSERTDEB_NOT_EQUAL_(__A, __B) ASSERT_NOT_EQUAL_(__A, __B)
#define ASSERTDEB_LT_(__A, __B) ASSERT_LT_(__A, __B)
#define ASSERTDEB_GT_(__A, __B) ASSERT_GT_(__A, __B)
#define ASSERTDEB_LE_(__A, __B) ASSERT_LE_(__A, __B)
#define ASSERTDEB_GE_(__A, __B) ASSERT_GE_(__A, __B)
#else
// clang-format off
#define ASSERTDEB_(f) while (0){}
#define ASSERTDEBMSG_(f, __ERROR_MSG) while (0){}
#define ASSERTDEB_EQUAL_(__A, __B) while (0){}
#define ASSERTDEB_NOT_EQUAL_(__A, __B) while (0){}
#define ASSERTDEB_LT_(__A, __B) while (0){}
#define ASSERTDEB_GT_(__A, __B) while (0){}
#define ASSERTDEB_LE_(__A, __B) while (0){}
#define ASSERTDEB_GE_(__A, __B) while (0){}
// clang-format on

#endif

/** The start of a standard MRPT "try...catch()" block that allows
 * tracing throw the call stack after an exception. \sa
 * MRPT_TRY_END,MRPT_TRY_END_WITH_CLEAN_UP
 */
#define MRPT_TRY_START \
	try                \
	{
/** The end of a standard MRPT "try...catch()" block that allows tracing
 * throw the call stack after an exception. \sa
 * MRPT_TRY_START,MRPT_TRY_END_WITH_CLEAN_UP
 */
#define MRPT_TRY_END                   \
	}                                  \
	catch (std::bad_alloc&) { throw; } \
	catch (...) { THROW_STACKED_EXCEPTION; }

/** The end of a standard MRPT "try...catch()" block that allows tracing
 * throw the call stack after an exception, including a "clean up" piece
 * of code to be run before throwing the exceptions. \sa
 * MRPT_TRY_END,MRPT_TRY_START
 *
 */
#define MRPT_TRY_END_WITH_CLEAN_UP(stuff) \
	}                                     \
	catch (std::bad_alloc&) { throw; }    \
	catch (...) { {stuff} THROW_STACKED_EXCEPTION; }

#if MRPT_ENABLE_EMBEDDED_GLOBAL_PROFILER
#define MRPT_PROFILE_FUNC_START                \
	::mrpt::system::CProfilerProxy BOOST_JOIN( \
		__dum_var, __LINE__)(__CURRENT_FUNCTION_NAME__);
#else
#define MRPT_PROFILE_FUNC_START
#endif

// General macros for use within each MRPT method/function. They
// provide:
//  - Nested exception handling
//  - Automatic profiling stats (in Debug only)
// ---------------------------------------------------------
#define MRPT_START          \
	MRPT_PROFILE_FUNC_START \
	MRPT_TRY_START

#define MRPT_END MRPT_TRY_END

#define MRPT_END_WITH_CLEAN_UP(stuff) MRPT_TRY_END_WITH_CLEAN_UP(stuff)
}  // namespace mrpt
