/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2025, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/core/constexpr_for.h>

TEST(constexpr_for, compileTest)
{
  std::size_t s = 0;
  mrpt::for_<10>([&](auto i) { s += i.value; });
  EXPECT_EQ(s, static_cast<size_t>((10 / 2) * 9));
}
