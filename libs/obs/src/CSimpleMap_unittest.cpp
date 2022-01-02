/* +------------------------------------------------------------------------+
|                     Mobile Robot Programming Toolkit (MRPT)            |
|                          https://www.mrpt.org/                         |
|                                                                        |
| Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
| See: https://www.mrpt.org/Authors - All rights reserved.               |
| Released under BSD License. See: https://www.mrpt.org/License          |
+------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/maps/CSimpleMap.h>
#include <test_mrpt_common.h>

TEST(CSimpleMap, ParseFileInFormat_v1_5)
{
	const std::string fil = mrpt::UNITTEST_BASEDIR +
		std::string("/share/mrpt/datasets/localization_demo.simplemap.gz");

	mrpt::maps::CSimpleMap sm;
	const bool load_ok = sm.loadFromFile(fil);
	EXPECT_TRUE(load_ok);
	EXPECT_EQ(sm.size(), 72U);
}
