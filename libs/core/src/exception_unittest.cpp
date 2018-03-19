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

TEST(exception, stackedException)
{
	EXPECT_THROW({
		THROW_STACKED_EXCEPTION(std::runtime_error("stacked"));
	}, std::logic_error);
}

TEST(exception, assertException)
{
	bool trueValue = true;
	bool falseValue = false;
	EXPECT_THROW({
		ASSERT_EQUAL_(trueValue, falseValue);
	}, std::logic_error);
}

TEST(exception, stackedExceptionCustomMsg)
{
	EXPECT_THROW({
		THROW_STACKED_EXCEPTION_CUSTOM_MSG2(std::runtime_error("stacked"),
			"Foo %s\n", "bar");
	}, std::logic_error);
}
