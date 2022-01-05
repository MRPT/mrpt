/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/maps/CPointsMapXYZI.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/CObservationRotatingScan.h>
#include <mrpt/obs/CObservationVelodyneScan.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/system/filesystem.h>
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

TEST(CObservationRotatingScan, fromVelodyne)
{
	using namespace std::string_literals;
	const auto fil = mrpt::system::getShareMRPTDir() +
		"/datasets/test_velodyne_VLP16.rawlog"s;

	mrpt::obs::CRawlog rawlog;
	bool load_ok = rawlog.loadFromRawLogFile(fil);
	EXPECT_TRUE(load_ok) << "Could not load " << fil;

	auto oVelo = rawlog.asObservation<mrpt::obs::CObservationVelodyneScan>(0);
	EXPECT_TRUE(oVelo.get() != nullptr);

	mrpt::obs::CObservationRotatingScan rs;
	rs.fromVelodyne(*oVelo);

	EXPECT_TRUE(rs.azimuthSpan > 0) << "Rotation: CCW";
	EXPECT_NEAR(std::abs(rs.azimuthSpan), 2 * M_PI, 0.1);
	EXPECT_EQ(rs.columnCount, 28800);
	EXPECT_EQ(rs.rowCount, 16);

	// std::cout << rs.getDescriptionAsTextValue();
}

TEST(CObservationRotatingScan, from2DScan)
{
	using namespace std::string_literals;
	const auto fil =
		mrpt::system::getShareMRPTDir() + "/datasets/localization_demo.rawlog"s;

	mrpt::obs::CRawlog rawlog;
	bool load_ok = rawlog.loadFromRawLogFile(fil);
	EXPECT_TRUE(load_ok) << "Could not load " << fil;

	auto oScan2D =
		rawlog.getAsObservations(1)
			->getObservationByClass<mrpt::obs::CObservation2DRangeScan>();
	EXPECT_TRUE(oScan2D.get() != nullptr);

	mrpt::obs::CObservationRotatingScan rs;
	rs.fromScan2D(*oScan2D);

	EXPECT_TRUE(rs.azimuthSpan > 0) << "Rotation: CCW";
	EXPECT_NEAR(std::abs(rs.azimuthSpan), M_PI, 1e-4);
	EXPECT_EQ(rs.columnCount, 361);
	EXPECT_EQ(rs.rowCount, 1);

	// std::cout << rs.getDescriptionAsTextValue();
}
