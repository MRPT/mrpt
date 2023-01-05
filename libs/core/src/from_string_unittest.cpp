/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/core/from_string.h>

// Load data from constant file and check for exact match.
TEST(FromStringTests, Basic)
{
	EXPECT_EQ(mrpt::from_string<double>("2.0"), 2.0);
	EXPECT_EQ(mrpt::from_string<double>("", 3.0, false), 3.0);
	EXPECT_EQ(mrpt::from_string<double>("abc", 3.0, false), 3.0);

	EXPECT_EQ(mrpt::from_string<int>("42"), 42);
	EXPECT_EQ(mrpt::from_string<int>("", 42, false), 42);

	EXPECT_EQ(mrpt::from_string<std::string>("abc"), "abc");
	EXPECT_EQ(mrpt::from_string<std::string>("abc def"), "abc def");
	EXPECT_EQ(
		mrpt::from_string<std::string>("<J2L2}h[p#{`@'?y"), "<J2L2}h[p#{`@'?y");
	EXPECT_EQ(
		mrpt::from_string<std::string>("[1.0 2.0;5.0 6.0]"),
		"[1.0 2.0;5.0 6.0]");
}
