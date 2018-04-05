/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include <mrpt/containers/with.h>
#include <mrpt/core/common.h>

#include <gtest/gtest.h>

int counter = 0;
struct MyType
{
	void foo() { ++counter; }
};

TEST(containers_with, call_all)
{
	MyType a, b, c, d;
	mrpt::containers::with(a, b, c, d).call([&](auto obj) -> void {
		obj.foo();
	});
	EXPECT_EQ(counter, 4);
}
