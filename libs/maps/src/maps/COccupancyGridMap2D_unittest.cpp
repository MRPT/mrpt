/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/stock_observations.h>
//
#include <mrpt/config.h>
#include <test_mrpt_common.h>

using namespace mrpt;
using namespace mrpt::maps;
using namespace mrpt::obs;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace std;

TEST(COccupancyGridMap2DTests, insert2DScan)
{
	mrpt::obs::CObservation2DRangeScan scan1;
	stock_observations::example2DRangeScan(scan1);

	// Insert the scan in the grid map and check expected values:
	{
		COccupancyGridMap2D grid(-50.0f, 50.0f, -50.0f, 50.0f, 0.10f);
		grid.insertObservation(scan1);

		EXPECT_GT(grid.getPos(0.5, 0), 0.51f);	// A cell in front of the laser
		// should have a high "freeness"
	}
}

// We need OPENCV to read the image.
#if MRPT_HAS_OPENCV && MRPT_HAS_FYAML

TEST(COccupancyGridMap2DTests, loadFromROSMapServerYAML)
{
	using namespace std::string_literals;
	const auto fil = mrpt::UNITTEST_BASEDIR() + "/tests/yaml_32.yaml"s;

	auto grid = mrpt::maps::COccupancyGridMap2D::FromROSMapServerYAML(fil);

	ASSERT_EQUAL_(grid.getResolution(), 0.15f);
	ASSERT_NEAR_(grid.getXMin(), -4.5f, 0.01f);
	ASSERT_NEAR_(grid.getXMax(), 0.f, 0.01f);
	ASSERT_NEAR_(grid.getYMin(), 0.f, 0.01f);
	ASSERT_NEAR_(grid.getYMax(), 9.9f, 0.01f);

	ASSERT_EQUAL_(grid.getPos(2.0, 2.0), 0.5f);
}

#endif