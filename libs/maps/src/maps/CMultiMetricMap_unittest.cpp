/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/maps/CMultiMetricMap.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <test_mrpt_common.h>

TEST(CMultiMetricMapTests, isEmpty)
{
	{
		mrpt::maps::CMultiMetricMap m;
		EXPECT_TRUE(m.isEmpty());
	}
}

static mrpt::maps::CMultiMetricMap initializer1()
{
	mrpt::maps::TSetOfMetricMapInitializers map_inits;
	{
		{
			mrpt::maps::COccupancyGridMap2D::TMapDefinition def;
			def.resolution = 0.05;
			def.insertionOpts.maxOccupancyUpdateCertainty = 0.8;
			def.insertionOpts.maxDistanceInsertion = 30;
			map_inits.push_back(def);
		}
		{
			mrpt::maps::CSimplePointsMap::TMapDefinition def;
			map_inits.push_back(def);
		}
		mrpt::maps::CMultiMetricMap m;
		m.setListOfMaps(map_inits);
		return m;
	}
}

static mrpt::maps::CMultiMetricMap initializer2()
{
	mrpt::maps::CMultiMetricMap m;
	m.maps.push_back(mrpt::maps::COccupancyGridMap2D::Create());
	m.maps.push_back(mrpt::maps::CSimplePointsMap::Create());
	return m;
}

TEST(CMultiMetricMapTests, initializers)
{
	{
		const auto m = initializer1();
		EXPECT_EQ(m.maps.size(), 2U);
		EXPECT_TRUE(IS_CLASS(*m.maps.at(0), mrpt::maps::COccupancyGridMap2D));
		EXPECT_TRUE(IS_CLASS(*m.maps.at(1), mrpt::maps::CSimplePointsMap));
	}
	{
		const auto m = initializer2();
		EXPECT_EQ(m.maps.size(), 2U);
		EXPECT_TRUE(IS_CLASS(*m.maps.at(0), mrpt::maps::COccupancyGridMap2D));
		EXPECT_TRUE(IS_CLASS(*m.maps.at(1), mrpt::maps::CSimplePointsMap));
	}
}

TEST(CMultiMetricMapTests, copyCtorOp)
{
	using mrpt::maps::CSimplePointsMap;

	const auto m1 = initializer1();
	EXPECT_EQ(m1.maps.size(), 2U);

	m1.mapByClass<CSimplePointsMap>()->insertPoint(1.0f, 2.0f, 3.0f);

	// Test deep copy:
	{
		mrpt::maps::CMultiMetricMap m2 = m1;

		EXPECT_EQ(m1.mapByClass<CSimplePointsMap>()->size(), 1U);
		EXPECT_EQ(m2.mapByClass<CSimplePointsMap>()->size(), 1U);

		m1.mapByClass<CSimplePointsMap>()->insertPoint(1.0f, 2.0f, 3.0f);

		EXPECT_EQ(m1.mapByClass<CSimplePointsMap>()->size(), 2U);
		EXPECT_EQ(m2.mapByClass<CSimplePointsMap>()->size(), 1U);
	}
}

TEST(CMultiMetricMapTests, moveOp)
{
	using mrpt::maps::CSimplePointsMap;

	const auto m1 = initializer1();
	EXPECT_EQ(m1.maps.size(), 2U);

	m1.mapByClass<CSimplePointsMap>()->insertPoint(1.0f, 2.0f, 3.0f);

	mrpt::maps::CMultiMetricMap m2 = std::move(m1);

	EXPECT_EQ(m2.mapByClass<CSimplePointsMap>()->size(), 1U);
}
