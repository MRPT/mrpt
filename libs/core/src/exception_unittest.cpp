/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
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
		const auto num_lines = std::count(sExc.begin(), sExc.end(), '\n');
		EXPECT_GT(num_lines, 10);

		EXPECT_TRUE(sExc.find("Message:  Aw!") != std::string::npos) << sExc;
		EXPECT_TRUE(sExc.find("test_except_2nd_lvl") != std::string::npos)
			<< sExc;
	}
}

TEST(exception, assertException)
{
	bool trueValue = true;
	bool falseValue = false;
	EXPECT_THROW(
		{ ASSERT_EQUAL_(trueValue, falseValue); },
		mrpt::ExceptionWithCallBackBase);
}
