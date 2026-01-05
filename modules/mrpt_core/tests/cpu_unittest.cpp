/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2025, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

#include <gtest/gtest.h>
#include <mrpt/core/cpu.h>

TEST(cpu, features_as_string)
{
  const std::string s = mrpt::cpu::features_as_string();
  std::cout << "CPU features: " << s << "\n";

  EXPECT_GT(s.size(), 10UL);
}

TEST(cpu, supports)
{
  // Test that, at least, it does not crash.
  // We cannot test against any known value since the result is CPU-dependant
  const bool r = mrpt::cpu::supports(mrpt::cpu::feature::SSE2);
  (void)r;
}
