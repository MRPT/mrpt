/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include <mrpt/maps/CPointsMapXYZI.h>
#include <mrpt/system/filesystem.h>
#include <gtest/gtest.h>
#include <iostream>

// Defined in tests/test_main.cpp
namespace mrpt
{
extern std::string MRPT_GLOBAL_UNITTEST_SRC_DIR;
}

TEST(CPointsMapXYZI, loadFromKittiVelodyneFile)
{
	using namespace std;
	const string kitti_fil = mrpt::MRPT_GLOBAL_UNITTEST_SRC_DIR +
							 string("/tests/kitti_00_000000.bin.gz");
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
