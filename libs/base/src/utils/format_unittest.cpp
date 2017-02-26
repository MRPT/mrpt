/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/utils/utils_defs.h>
#include <gtest/gtest.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace std;


// Load data from constant file and check for exact match.
TEST(FormatTest, LargeStrings)
{
	string   test_str;
	const size_t test_str_len = 30000;
	test_str.assign(test_str_len,'A');

	std::string s =	mrpt::format("%u %s",static_cast<unsigned int>(10), test_str.c_str() );

	const size_t out_str_len = s.size();

	// If it works, out len must be that of the string, plus 3 from "10 "
	EXPECT_EQ(out_str_len, (test_str_len+3) );
}

