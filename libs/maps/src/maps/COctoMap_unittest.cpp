/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/maps/COctoMap.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/stock_observations.h>

using namespace mrpt;
using namespace mrpt::maps;
using namespace mrpt::obs;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace std;

TEST(COctoMapTests, updateVoxels)
{
	// Copied from the example program in the "octomap" C++ library.

	COctoMap map(0.1);

	map.updateVoxel(1, 1, 1, true);	 // integrate 'occupied' measurement

	map.updateVoxel(1.5, 1, 1, true);  // integrate 'occupied' measurement
	map.updateVoxel(1.5, 1, 1, true);  // integrate 'occupied' measurement
	map.updateVoxel(1.5, 1, 1, true);  // integrate 'occupied' measurement

	map.updateVoxel(-1, -1, 1, false);	// integrate 'occupied' measurement

	double occup;
	bool is_mapped;
	mrpt::math::TPoint3D pt;

	pt = mrpt::math::TPoint3D(1, 1, 1);
	is_mapped = map.getPointOccupancy(pt.x, pt.y, pt.z, occup);
	EXPECT_GT(occup, 0.5);
	EXPECT_TRUE(is_mapped);

	pt = mrpt::math::TPoint3D(-1, -1, 1);
	is_mapped = map.getPointOccupancy(pt.x, pt.y, pt.z, occup);
	EXPECT_LT(occup, 0.5);
	EXPECT_TRUE(is_mapped);
}

TEST(COctoMapTests, insert2DScan)
{
	// Load scans:
	mrpt::obs::CObservation2DRangeScan scan1;
	stock_observations::example2DRangeScan(scan1);

	// Insert the scan in the map and check expected values:
	{
		COctoMap map(0.1);
		map.insertObservation(scan1);
	}
}
