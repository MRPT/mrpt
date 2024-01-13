/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/maps/CPointsMapXYZI.h>
#include <mrpt/system/filesystem.h>
#include <test_mrpt_common.h>

#include <iostream>

TEST(CPointsMapXYZI, loadFromKittiVelodyneFile)
{
	using namespace std;
	const string kitti_fil =
		mrpt::UNITTEST_BASEDIR() + string("/tests/kitti_00_000000.bin.gz");
	if (!mrpt::system::fileExists(kitti_fil))
	{
		cerr << "WARNING: Skipping test due to missing file: " << kitti_fil
			 << "\n";
		return;
	}

	mrpt::maps::CPointsMapXYZI m;
	const bool read_ok = m.loadFromKittiVelodyneFile(kitti_fil);
	EXPECT_TRUE(read_ok);
	EXPECT_EQ(124668U, m.size());
}
