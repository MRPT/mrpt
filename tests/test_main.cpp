/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          https://www.mrpt.org/                            |
   |                                                                           |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file        |
   | See: https://www.mrpt.org/Authors - All rights reserved.                  |
   | Released under BSD License. See details in https://www.mrpt.org/License   |
   +---------------------------------------------------------------------------+
 */

#include <gtest/gtest.h>
#include <test_mrpt_common.h>

std::string mrpt::UNITTEST_BASEDIR = CMAKE_UNITTEST_BASEDIR;

int main(int argc, char** argv)
{
	testing::InitGoogleTest(&argc, argv);

	if (argc > 1) mrpt::UNITTEST_BASEDIR = std::string(argv[1]);

	return RUN_ALL_TESTS();
}
