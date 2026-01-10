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
#include <mrpt/maps/CLogOddsGridMapLUT.h>

#include <numeric>

template <typename cell_t>
void test_monotonic()
{
  mrpt::maps::CLogOddsGridMapLUT<cell_t> lut;
  float last_p = .0f;
  const auto i_init = std::numeric_limits<cell_t>::min();
  const auto i_end = std::numeric_limits<cell_t>::max();
  for (int64_t i = i_init; i <= i_end; i++)
  {
    // Ensure LUT is monotonic:
    const float new_p = lut.l2p(i);
    EXPECT_GE(new_p, last_p) << " i=" << i;
    last_p = new_p;
  }
  EXPECT_NEAR(lut.l2p(i_init), .0f, 0.05);
  EXPECT_NEAR(lut.l2p(i_end), 1.0f, 0.05);

  // Expect internal table to be monotonic:
  int64_t last_logodd = std::numeric_limits<int64_t>::min();
  for (size_t idx = 0; idx < lut.p2lTable.size(); idx++)
  {
    const int64_t next_logodd = lut.p2lTable[idx];
    EXPECT_GE(next_logodd, last_logodd) << "idx=" << idx;

    last_logodd = next_logodd;
  }
}

TEST(CLogOddsGridMapLUT, monotonic_8bit) { test_monotonic<int8_t>(); }

TEST(CLogOddsGridMapLUT, monotonic_16bit) { test_monotonic<int16_t>(); }
