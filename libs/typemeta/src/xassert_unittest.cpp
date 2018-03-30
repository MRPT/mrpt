/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include <mrpt/typemeta/xassert.h>
#include <gtest/gtest.h>

constexpr int foo_i_below_10(unsigned i) { return MRPT_X_ASSERT(i < 10), 0; }
TEST(XAssert, build_time)
{
	// Builds:
	constexpr int x = foo_i_below_10(0);
	(void)(x);

	// Does not build
	// constexpr int y = foo_i_below_10(11);

	// runs:
	foo_i_below_10(0);

	// throws:
	EXPECT_THROW(foo_i_below_10(11), std::exception);
}
