/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/maps/COccupancyGridMap3D.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/stock_observations.h>
#include <gtest/gtest.h>

TEST(COccupancyGridMap3DTests, insert2DScan)
{
	mrpt::obs::CObservation2DRangeScan scan1;
	stock_observations::example2DRangeScan(scan1);

	// Insert the scan in the grid map and check expected values:
	{
		mrpt::maps::COccupancyGridMap3D grid;
		grid.insertObservation(&scan1);

		// A cell in front of the laser should have a high "freeness"
		EXPECT_GT(grid.getPos(0.5, 0, 0), 0.51f);
	}
}
