/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>

// Reuse code from example:
#include "samples/math_optimize_lm_example/LevMarqTest_impl.cpp"

TEST(LevMarq, optimizeTest)
{
	TestLevMarq();
	EXPECT_LT(levmarq_final_error, 1e-2);
}
