/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/typemeta/xassert.h>

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
