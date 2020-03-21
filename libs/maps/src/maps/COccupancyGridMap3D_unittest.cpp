/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/maps/COccupancyGridMap3D.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/obs/stock_observations.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/filesystem.h>
#include <test_mrpt_common.h>

TEST(COccupancyGridMap3DTests, insert2DScan)
{
	mrpt::obs::CObservation2DRangeScan scan1;
	mrpt::obs::stock_observations::example2DRangeScan(scan1);

	// Insert the scan in the grid map and check expected values:
	{
		mrpt::maps::COccupancyGridMap3D grid;
		grid.insertObservation(scan1);

		// A cell in front of the laser should have a high "freeness"
		EXPECT_GT(grid.getFreenessByPos(0.5, 0, 0), 0.53f);
	}
}

// We need OPENCV to read the image internal to CObservation3DRangeScan,
// so skip this test if built without opencv.
#if MRPT_HAS_OPENCV

TEST(COccupancyGridMap3DTests, insertScan3D)
{
	using namespace std::string_literals;
	const auto fil =
		mrpt::UNITTEST_BASEDIR + "/tests/test-3d-obs-ground.rawlog"s;
	if (!mrpt::system::fileExists(fil))
	{
		GTEST_FAIL() << "ERROR: test due to missing file: " << fil << "\n";
		return;
	}

	// Load sample 3D scan from file:
	mrpt::obs::CSensoryFrame sf;
	mrpt::io::CFileGZInputStream f(fil);
	mrpt::serialization::archiveFrom(f) >> sf;

	auto obs = sf.getObservationByClass<mrpt::obs::CObservation3DRangeScan>();
	ASSERT_(obs);

	{
		mrpt::maps::COccupancyGridMap3D grid;
		grid.insertObservation(*obs);

		// A cell in front of the laser should have a high "freeness"
		EXPECT_GT(grid.getFreenessByPos(0.2f, 0.2f, 0.1f), 0.53f);
	}
}

#endif
