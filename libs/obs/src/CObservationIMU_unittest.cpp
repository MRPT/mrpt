/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/serialization/CArchive.h>
#include <test_mrpt_common.h>

static void checkExpectedValues(const mrpt::obs::CObservationIMU& o)
{
	EXPECT_FALSE(o.has(mrpt::obs::IMU_X));

	EXPECT_TRUE(o.has(mrpt::obs::IMU_WX));
	EXPECT_TRUE(o.has(mrpt::obs::IMU_WY));
	EXPECT_TRUE(o.has(mrpt::obs::IMU_WZ));

	EXPECT_NEAR(o.get(mrpt::obs::IMU_WX), 0.005555555, 1e-4);
	EXPECT_NEAR(o.get(mrpt::obs::IMU_WY), 0.0, 1e-4);
	EXPECT_NEAR(o.get(mrpt::obs::IMU_WZ), 0.00222222, 1e-4);
}

TEST(CObservationIMU, Deserialize_v3)
{
	using namespace std::string_literals;

	mrpt::obs::CRawlog dataset;

	//! JS_PRELOAD_FILE <tests/test-imu-obs-format3.rawlog>
	const auto sFil =
		mrpt::UNITTEST_BASEDIR + "/tests/test-imu-obs-format3.rawlog"s;

	bool loadOk = dataset.loadFromRawLogFile(sFil);
	EXPECT_TRUE(loadOk);
	EXPECT_EQ(dataset.size(), 1U);

	auto o = dataset.asObservation<mrpt::obs::CObservationIMU>(0);
	checkExpectedValues(*o);
}

TEST(CObservationIMU, Deserialize_v4)
{
	using namespace std::string_literals;

	mrpt::obs::CRawlog dataset;

	//! JS_PRELOAD_FILE <tests/test-imu-obs-format4.rawlog>
	const auto sFil =
		mrpt::UNITTEST_BASEDIR + "/tests/test-imu-obs-format4.rawlog"s;

	bool loadOk = dataset.loadFromRawLogFile(sFil);
	EXPECT_TRUE(loadOk);
	EXPECT_EQ(dataset.size(), 1U);

	auto o = dataset.asObservation<mrpt::obs::CObservationIMU>(0);
	checkExpectedValues(*o);
}
