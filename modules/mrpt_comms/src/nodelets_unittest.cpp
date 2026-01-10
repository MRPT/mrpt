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

// Reuse code from example:
//#define NODELETS_TEST_VERBOSE
#include "samples/comms_nodelets_example/NodeletsTest_impl.cpp"

TEST(NodeletsTests, pub_sub_multithread_test)
{
  NodeletsTest();
  EXPECT_TRUE(nodelets_test_passed_ok);
}
