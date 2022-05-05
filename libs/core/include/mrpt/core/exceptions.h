/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/core/abs_diff.h>	 // mrpt::abs_diff()
#include <mrpt/core/backtrace.h>
#include <mrpt/core/common.h>
#include <mrpt/core/format.h>

#include <cstdlib>	// std::abs
#include <stdexcept>  // logic_error
#include <string>  // std::string, to_string()
#include <string_view>

namespace mrpt::internal
{
int MAX_BACKTRACE_DEPTH();

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
/** Builds a nice textual representation of an exception, which if
 * generated using MRPT macros (THROW_EXCEPTION,...) in between
 * MRPT_START/MRPT_END macros, will contain function names and line numbers
 * across the call stack at the original throw point.
 * Since MRPT 2.1.5 this will also print line numbers of any other exception
 * (even if generated in non-mrpt code, like inside STL headers)
 * as long as MRPT was built with BFD support and sources built with, at least,
 * -g1 debug symbols.
 *
 * See examples of use in \ref mrpt_core_grp
 * \ingroup mrpt_core_grp
 */
std::string exception_to_str(const std::exception& e);

struct ExceptionWithCallBackBase
{
	ExceptionWithCallBackBase(
		const std::string original_what, const TCallStackBackTrace call_stack)
		: originalWhat(original_what), callStack(call_stack)
	{
	}

	const std::string originalWhat;
	const TCallStackBackTrace callStack;
};

/** A wrapper around an std::exception */
template <class BASE_EXCEPTION>
struct ExceptionWithCallBack : public BASE_EXCEPTION,
							   public ExceptionWithCallBackBase
{
	ExceptionWithCallBack(const BASE_EXCEPTION& originalException)
		: BASE_EXCEPTION(originalException),
		  ExceptionWithCallBackBase(
			  originalException.what(),
			  mrpt::callStackBackTrace(
				  2 /*skip 2 frames*/,
				  2 + mrpt::internal::MAX_BACKTRACE_DEPTH()))
	{
	}

	/** Use this pointer only before this object is destroyed */
	const char* what() const noexcept override
	{
		if (m_what.empty()) m_what = mrpt::exception_to_str(*this);
		return m_what.c_str();
	}

   private:
	mutable std::string m_what;
};

/** \def THROW_TYPED_EXCEPTION(msg,exceptionClass) */
#define THROW_TYPED_EXCEPTION(msg, exceptionClass)                             \
	throw mrpt::ExceptionWithCallBack<exceptionClass>(                         \
		exceptionClass(mrpt::internal::exception_line_msg(                     \
			msg, __FILE__, __LINE__, __CURRENT_FUNCTION_NAME__)))

/** \def THROW_EXCEPTION(msg);
 * \param msg This can be a char*, a std::string, or a literal string.
 * Defines a unified way of reporting exceptions
 * \sa MRPT_TRY_START, MRPT_TRY_END, THROW_EXCEPTION_FMT
 */
#define THROW_EXCEPTION(msg) THROW_TYPED_EXCEPTION(msg, std::logic_error)

#define THROW_EXCEPTION_FMT(_FORMAT_STRING, ...)                               \
	THROW_EXCEPTION(mrpt::format(_FORMAT_STRING, __VA_ARGS__))

#define THROW_TYPED_EXCEPTION_FMT(exceptionClass, _FORMAT_STRING, ...)         \
	THROW_TYPED_EXCEPTION(                                                     \
		mrpt::format(_FORMAT_STRING, __VA_ARGS__), exceptionClass)

/** For use in CSerializable implementations */
#define MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(__V)                          \
	THROW_EXCEPTION_FMT(                                                       \
		"Cannot parse object: unknown serialization version number: '%i'",     \
		static_cast<int>(__V))

/** Throws a stacked exception if condition "f" is false; with custom message.
 * \sa MRPT_TRY_START, MRPT_TRY_END
 */
#define ASSERTMSG_(f, __ERROR_MSG)                                             \
	do                                                                         \
	{                                                                          \
		if (!(f)) THROW_EXCEPTION(::std::string(__ERROR_MSG));                 \
	} while (0)

/** Throws a stacked exception if condition "f" is false.
 * \sa MRPT_TRY_START, MRPT_TRY_END
 */
#define ASSERT_(f)                                                             \
	ASSERTMSG_(f, std::string("Assert condition failed: ") + ::std::string(#f))

/** Throws an exception if the number is NaN, IND, or +/-INF, or return the same
 * number otherwise. */
#define MRPT_CHECK_NORMAL_NUMBER(v)                                            \
	do                                                                         \
	{                                                                          \
		ASSERT_(std::isfinite(v));                                             \
		ASSERT_(!std::isnan(v));                                               \
	} while (0)

#define ASRT_FAIL(__CONDITIONSTR, __A, __B, __ASTR, __BSTR)                    \
	THROW_EXCEPTION(                                                           \
		mrpt::internal::asrt_fail(__CONDITIONSTR, __A, __B, __ASTR, __BSTR));

/** Assert comparing two values, reporting their actual values upon failure */
#define ASSERT_EQUAL_(__A, __B)                                                \
	do                                                                         \
	{                                                                          \
		if (__A != __B) ASRT_FAIL("ASSERT_EQUAL_", __A, __B, #__A, #__B)       \
	} while (0)

#define ASSERT_NOT_EQUAL_(__A, __B)                                            \
	do                                                                         \
	{                                                                          \
		if (__A == __B) ASRT_FAIL("ASSERT_NOT_EQUAL_", __A, __B, #__A, #__B)   \
	} while (0)

/** Checks two float/double values, reporting their values upon failure.
 * \note If A, B, and Tolerance are of different types, keep in mind that they
 *       will be compared after converting into typeof(A).
 */
#define ASSERT_NEAR_(__A, __B, __TOLERANCE)                                    \
	do                                                                         \
	{                                                                          \
		const auto diff =                                                      \
			mrpt::abs_diff<std::decay_t<decltype(__A)>>((__A), (__B));         \
		if (diff > __TOLERANCE)                                                \
			ASRT_FAIL("ASSERT_NEAR_", __A, __B, #__A, #__B)                    \
	} while (0)

/** Checks A<B */
#define ASSERT_LT_(__A, __B)                                                   \
	do                                                                         \
	{                                                                          \
		if (__A >= __B) ASRT_FAIL("ASSERT_LT_", __A, __B, #__A, #__B)          \
	} while (0)

/** Checks A<=B */
#define ASSERT_LE_(__A, __B)                                                   \
	do                                                                         \
	{                                                                          \
		if (__A > __B) ASRT_FAIL("ASSERT_LE_", __A, __B, #__A, #__B)           \
	} while (0)

/** Checks A>B */
#define ASSERT_GT_(__A, __B)                                                   \
	do                                                                         \
	{                                                                          \
		if (__A <= __B) ASRT_FAIL("ASSERT_GT_", __A, __B, #__A, #__B)          \
	} while (0)

/** Checks A>=B */
#define ASSERT_GE_(__A, __B)                                                   \
	do                                                                         \
	{                                                                          \
		if (__A < __B) ASRT_FAIL("ASSERT_GE_", __A, __B, #__A, #__B)           \
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
#define MRPT_TRY_START                                                         \
	try                                                                        \
	{
/** The end of a standard MRPT "try...catch()" block that allows tracing
 * throw the call stack after an exception. \sa
 * MRPT_TRY_START,MRPT_TRY_END_WITH_CLEAN_UP
 */
#define MRPT_TRY_END                                                           \
	}                                                                          \
	catch (std::bad_alloc&) { throw; }                                         \
	catch (const mrpt::ExceptionWithCallBackBase&) { throw; }                  \
	catch (const std::exception& __e)                                          \
	{                                                                          \
		throw mrpt::ExceptionWithCallBack(__e);                                \
	}

/** The end of a standard MRPT "try...catch()" block that allows tracing
 * throw the call stack after an exception, including a "clean up" piece
 * of code to be run before throwing the exceptions. \sa
 * MRPT_TRY_END,MRPT_TRY_START
 *
 */
#define MRPT_TRY_END_WITH_CLEAN_UP(stuff)                                      \
	}                                                                          \
	catch (std::bad_alloc&) { throw; }                                         \
	catch (...)                                                                \
	{                                                                          \
		{                                                                      \
			stuff                                                              \
		}                                                                      \
		throw;                                                                 \
	}

// General macros for use within each MRPT method/function. They
// provide nested exception handling
#define MRPT_START MRPT_TRY_START
#define MRPT_END MRPT_TRY_END
#define MRPT_END_WITH_CLEAN_UP(stuff) MRPT_TRY_END_WITH_CLEAN_UP(stuff)

}  // namespace mrpt
