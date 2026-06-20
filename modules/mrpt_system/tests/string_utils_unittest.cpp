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
#include <mrpt/system/string_utils.h>

TEST(string_utils, firstNLines)
{
  const std::string s = "1\n2\n3\n4\n";

  EXPECT_EQ(mrpt::system::firstNLines(s, 1), "1");
  EXPECT_EQ(mrpt::system::firstNLines(s, 2), "1\n2");
  EXPECT_EQ(mrpt::system::firstNLines(s, 3), "1\n2\n3");
  EXPECT_EQ(mrpt::system::firstNLines(s, 10), s);
}
