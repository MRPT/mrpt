/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/core/is_defined.h>

struct A
{
	int x;
};
struct B;

TEST(is_defined, tests1)
{
	EXPECT_TRUE(mrpt::is_defined_v<A>);
	EXPECT_FALSE(mrpt::is_defined_v<B>);
}
