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
	EXPECT_THROW({ THROW_STACKED_EXCEPTION; }, std::logic_error);
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
		// std::cerr << sExc;
		const auto num_lines = std::count(sExc.begin(), sExc.end(), '\n');
		EXPECT_EQ(num_lines, 6);
	}
}

TEST(exception, assertException)
{
	bool trueValue = true;
	bool falseValue = false;
	EXPECT_THROW({ ASSERT_EQUAL_(trueValue, falseValue); }, std::logic_error);
}

TEST(exception, stackedExceptionCustomMsg)
{
	EXPECT_THROW(
		{ THROW_STACKED_EXCEPTION_CUSTOM_MSG2("Foo %s\n", "bar"); },
		std::logic_error);
}
