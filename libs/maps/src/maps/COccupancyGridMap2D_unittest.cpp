/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/stock_observations.h>

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

		EXPECT_GT(grid.getPos(0.5, 0), 0.51f);  // A cell in front of the laser
		// should have a high "freeness"
	}
}
