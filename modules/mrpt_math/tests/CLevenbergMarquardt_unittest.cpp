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

// Reuse code from example:
#include "../../../samples/math_optimize_lm_example/LevMarqTest_impl.cpp"  // NOLINT

TEST(LevMarq, optimizeTest)
{
  TestLevMarq();
  EXPECT_LT(levmarq_final_error, 1e-2);
}
