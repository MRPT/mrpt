/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/config.h>
#include <mrpt/core/exceptions.h>

#include <algorithm>  // count()
#include <sstream>

TEST(exception, stackedExceptionBasic)
{
	EXPECT_THROW({ THROW_EXCEPTION("wtf"); }, mrpt::ExceptionWithCallBackBase);
}

template <typename T>
void test_except_3rd_lvl(T val)
{
	MRPT_START
	THROW_EXCEPTION("Aw!");
	val++;
	MRPT_END
}

void test_except_2nd_lvl()
{
	MRPT_START
	test_except_3rd_lvl<int>(10);
	MRPT_END
}

void test_except_toplevel()
{
	MRPT_START
	test_except_2nd_lvl();
	MRPT_END
}

#if !MRPT_IN_EMSCRIPTEN
TEST(exception, stackedExceptionComplex)
{
	try
	{
		test_except_toplevel();
		GTEST_FAIL() << "Shouldn't reach here.";
	}
	catch (const std::exception& e)
	{
		const auto sExc = mrpt::exception_to_str(e);

#if !defined(MRPT_EXCEPTIONS_WITH_CALL_STACK)
		EXPECT_TRUE(sExc.find("Aw!") != std::string::npos) << sExc;
#else
		const auto num_lines = std::count(sExc.begin(), sExc.end(), '\n');
		EXPECT_GT(num_lines, 10) << sExc;
		EXPECT_TRUE(sExc.find("Message:  Aw!") != std::string::npos) << sExc;
#endif
// This test doesn't pass in Windows if building w/o debug symbols:
#if defined(MRPT_EXCEPTIONS_WITH_CALL_STACK) &&                                \
	(!defined(_WIN32) || defined(_DEBUG))
		EXPECT_TRUE(sExc.find("test_except_toplevel") != std::string::npos)
			<< sExc;
#endif
	}
}
#endif

TEST(exception, assertException)
{
	bool trueValue = true;
	bool falseValue = false;
	EXPECT_THROW(
		{ ASSERT_EQUAL_(trueValue, falseValue); },
		mrpt::ExceptionWithCallBackBase);
}

static std::string testFoo()
{
	try
	{
		std::vector<int> dummy;
		ASSERTMSG_(!dummy.empty(), "Check it");
		return {};
	}
	catch (std::exception& e)
	{
		const auto err = mrpt::exception_to_str(e);
		return err;
	}
}

TEST(exception, infiniteRecurseBug)
{
	// Should not crash:
	const auto s = testFoo();
	EXPECT_TRUE(s.find("Check it") != std::string::npos);
}

TEST(exception, assertMacros)
{
	// ASSERT_EQUAL_
	EXPECT_NO_THROW(ASSERT_EQUAL_(1, 1));
	EXPECT_NO_THROW(ASSERT_EQUAL_(1 + 3, 1 + 1 + 1 + 1));
	EXPECT_ANY_THROW(ASSERT_EQUAL_(1, 2));
	EXPECT_ANY_THROW(ASSERT_EQUAL_(1 + 3, 3 + 2));

	// ASSERT_NOT_EQUAL_
	EXPECT_ANY_THROW(ASSERT_NOT_EQUAL_(1, 1));
	EXPECT_ANY_THROW(ASSERT_NOT_EQUAL_(1 + 3, 1 + 1 + 1 + 1));
	EXPECT_NO_THROW(ASSERT_NOT_EQUAL_(1, 2));
	EXPECT_NO_THROW(ASSERT_NOT_EQUAL_(1 + 3, 3 + 2));

	// ASSERT_NEAR_
	EXPECT_NO_THROW(ASSERT_NEAR_(10.0, 10.01, 0.2));
	EXPECT_NO_THROW(ASSERT_NEAR_(10.0 + 1.0, 10.0 + 1.01, 0.2));
	EXPECT_NO_THROW(ASSERT_NEAR_(1, 1, 0));
	EXPECT_ANY_THROW(ASSERT_NEAR_(1, 2, 0));
	EXPECT_ANY_THROW(ASSERT_NEAR_(10.0, 10.01, 0.001));
	EXPECT_ANY_THROW(ASSERT_NEAR_(10.0, 10.01f, 0.001));
	EXPECT_ANY_THROW(ASSERT_NEAR_(10.0, 10.01f, 0.001f));
	EXPECT_ANY_THROW(ASSERT_NEAR_(10.0f, 10.01, 0.001f));
	EXPECT_NO_THROW(ASSERT_NEAR_(10.0, 10.01f, 0.1));
	EXPECT_NO_THROW(ASSERT_NEAR_(10.0f, 10.01f, 0.1));
	EXPECT_NO_THROW(ASSERT_NEAR_(10.0f, 10.01, 0.1));
	EXPECT_NO_THROW(ASSERT_NEAR_(10.0, 10.01f, 0.1f));

	// ASSERT_LT_
	EXPECT_NO_THROW(ASSERT_LT_(1.0, 2.0));
	EXPECT_ANY_THROW(ASSERT_LT_(3, 2));
	EXPECT_ANY_THROW(ASSERT_LT_(-1, -3));

	// ASSERT_LE_
	EXPECT_NO_THROW(ASSERT_LE_(1.0, 2.0));
	EXPECT_NO_THROW(ASSERT_LE_(1.0, 1.0));

	// ASSERT_GT_
	EXPECT_ANY_THROW(ASSERT_GT_(1.0, 2.0));
	EXPECT_NO_THROW(ASSERT_GT_(3, 2));
	EXPECT_NO_THROW(ASSERT_GT_(-1, -3));

	// ASSERT_GE_
	EXPECT_NO_THROW(ASSERT_GE_(2.0, 1.0));
	EXPECT_NO_THROW(ASSERT_GE_(1.0, 1.0));
}
