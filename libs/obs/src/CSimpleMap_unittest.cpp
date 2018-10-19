/* +------------------------------------------------------------------------+
|                     Mobile Robot Programming Toolkit (MRPT)            |
|                          http://www.mrpt.org/                          |
|                                                                        |
| Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
| See: http://www.mrpt.org/Authors - All rights reserved.                |
| Released under BSD License. See details in http://www.mrpt.org/License |
+------------------------------------------------------------------------+ */

#include <mrpt/maps/CSimpleMap.h>
#include <gtest/gtest.h>

// Defined in tests/test_main.cpp
namespace mrpt
{
extern std::string MRPT_GLOBAL_UNITTEST_SRC_DIR;
}

TEST(CSimpleMap, ParseFileInFormat_v1_5)
{
#if MRPT_IS_BIG_ENDIAN
	MRPT_TODO("Debug this issue in big endian platforms")
	return;  // Skip this test for now
#endif

	const std::string fil =
		mrpt::MRPT_GLOBAL_UNITTEST_SRC_DIR +
		std::string("/share/mrpt/datasets/localization_demo.simplemap.gz");

	mrpt::maps::CSimpleMap sm;
	const bool load_ok = sm.loadFromFile(fil);
	EXPECT_TRUE(load_ok);
	EXPECT_EQ(sm.size(), 72U);
}
