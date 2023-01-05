/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/system/string_utils.h>

TEST(string_utils, firstNLines)
{
	const std::string s = "1\n2\n3\n4\n";

	EXPECT_EQ(mrpt::system::firstNLines(s, 1), "1");
	EXPECT_EQ(mrpt::system::firstNLines(s, 2), "1\n2");
	EXPECT_EQ(mrpt::system::firstNLines(s, 3), "1\n2\n3");
	EXPECT_EQ(mrpt::system::firstNLines(s, 10), s);
}
