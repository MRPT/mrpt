/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

#include <gtest/gtest.h>
#include <mrpt/containers/visit_each.h>
#include <mrpt/core/common.h>

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
