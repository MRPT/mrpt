/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include <mrpt/core/exceptions.h>
#include <gtest/gtest.h>
#include <sstream>
#include <algorithm>  // count()

TEST(exception, stackedExceptionBasic)
{
	EXPECT_THROW({ THROW_STACKED_EXCEPTION; }, std::logic_error);
}

void test_except_3rd_lvl()
{
	MRPT_START
	THROW_EXCEPTION("Aw!");
	MRPT_END
}

void test_except_2nd_lvl()
{
	MRPT_START
	test_except_3rd_lvl();
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
		EXPECT_EQ(num_lines, 5);
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
