/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <gtest/gtest.h>

using namespace std;

namespace mrpt { namespace utils {
	std::string MRPT_GLOBAL_UNITTEST_SRC_DIR = CMAKE_MRPT_GLOBAL_UNITTEST_SRC_DIR;
  }
}

int main(int argc, char **argv)
{
	testing::InitGoogleTest(&argc, argv);

	if (argc>1) mrpt::utils::MRPT_GLOBAL_UNITTEST_SRC_DIR=std::string(argv[1]);

	return RUN_ALL_TESTS();
}
