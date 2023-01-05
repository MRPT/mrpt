/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/core/format.h>

// Load data from constant file and check for exact match.
TEST(FormatTest, LargeStrings)
{
	std::string test_str;
	const size_t test_str_len = 30000;
	test_str.assign(test_str_len, 'A');

	std::string s =
		mrpt::format("%u %s", static_cast<unsigned int>(10), test_str.c_str());

	const size_t out_str_len = s.size();

	// If it works, out len must be that of the string, plus 3 from "10 "
	EXPECT_EQ(out_str_len, (test_str_len + 3));
}

TEST(FormatTest, ToString)
{
	EXPECT_EQ(mrpt::to_string(false), "false");
	EXPECT_EQ(mrpt::to_string(true), "true");
}
