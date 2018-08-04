/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>

// Reuse code from example:
//#define NODELETS_TEST_VERBOSE
#include "samples/comms_nodelets_example/NodeletsTest_impl.cpp"

TEST(NodeletsTests, pub_sub_multithread_test)
{
	NodeletsTest();
	EXPECT_TRUE(nodelets_test_passed_ok);
}
