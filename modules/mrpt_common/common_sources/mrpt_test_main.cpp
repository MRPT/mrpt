/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          https://www.mrpt.org/                            |
   |                                                                           |
   | Copyright (c) 2005-2026, Individual contributors, see AUTHORS file        |
   | See: https://www.mrpt.org/Authors - All rights reserved.                  |
   | Released under BSD License. See details in https://www.mrpt.org/License   |
   +---------------------------------------------------------------------------+
 */

#include <gtest/gtest.h>
#include <test_mrpt_common.h>

std::string mrpt::UNITTEST_BASEDIR() { return CMAKE_UNITTEST_BASEDIR; }

#ifdef MRPT_DATA_DIR
static const std::string mrpt_data_dir_value = MRPT_DATA_DIR;
#undef MRPT_DATA_DIR
std::string mrpt::mrpt_data_dir() { return mrpt_data_dir_value; }
#else
std::string mrpt::mrpt_data_dir() { return CMAKE_UNITTEST_BASEDIR; }
#endif

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
