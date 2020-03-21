/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/containers/visit_each.h>
#include <mrpt/core/common.h>

#include <gtest/gtest.h>

int counter = 0;
struct Bar1
{
	void foo() { ++counter; }
};
struct Bar2
{
	void foo() { ++counter; }
};

TEST(containers_visit_each, call_all)
{
	Bar1 a, b;
	Bar2 c, d;
	mrpt::visit_each([&](auto obj) -> void { obj.foo(); }, a, b, c, d);
	EXPECT_EQ(counter, 4);
}
