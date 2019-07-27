/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/maps/CPointsMapXYZI.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/CObservationRotatingScan.h>
#include <test_mrpt_common.h>

TEST(CObservationRotatingScan, fromKittiUndistorted)
{
	using namespace std::string_literals;
	const auto fil = mrpt::UNITTEST_BASEDIR + "/tests/kitti_00_000000.bin.gz"s;

	auto pts = mrpt::maps::CPointsMapXYZI::Create();
	bool read_ok = pts->loadFromKittiVelodyneFile(fil);
	EXPECT_TRUE(read_ok);

	//	read_ok = pts->loadXYZI_from_text_file("0000000060.txt");
	EXPECT_TRUE(read_ok);

	pts->saveToKittiVelodyneFile("/tmp/a.bin.gz");
	pts->saveXYZI_to_text_file("/tmp/a.txt");

	mrpt::obs::CObservationPointCloud obsPcl;
	//	obsPcl.

	mrpt::obs::CObservationRotatingScan rs;
	//	rs.fromPointCloud()
}
