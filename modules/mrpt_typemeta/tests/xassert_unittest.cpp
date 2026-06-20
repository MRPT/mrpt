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
