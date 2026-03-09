/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

#include <gtest/gtest.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservationRotatingScan.h>
#include <mrpt/obs/CObservationVelodyneScan.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/system/filesystem.h>
#include <test_mrpt_common.h>

TEST(CObservationRotatingScan, fromVelodyne)
{
  using namespace std::string_literals;
  // TODO: Update to use mrpt_data package?
  // const auto fil = mrpt::system::getShareMRPTDir() + "/datasets/test_velodyne_VLP16.rawlog"s;
  const auto fil = mrpt::mrpt_data_dir() + "/datasets/test_velodyne_VLP16.rawlog"s;

  ASSERT_FILE_EXISTS_(fil);

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
  // TODO: Update to use mrpt_data package?
  // const auto fil = mrpt::system::getShareMRPTDir() + "/datasets/localization_demo.rawlog"s;
  const auto fil = mrpt::mrpt_data_dir() + "/datasets/localization_demo.rawlog"s;
  ASSERT_FILE_EXISTS_(fil);

  mrpt::obs::CRawlog rawlog;
  bool load_ok = rawlog.loadFromRawLogFile(fil);
  EXPECT_TRUE(load_ok) << "Could not load " << fil;

  auto oScan2D =
      rawlog.getAsObservations(1)->getObservationByClass<mrpt::obs::CObservation2DRangeScan>();
  EXPECT_TRUE(oScan2D.get() != nullptr);

  mrpt::obs::CObservationRotatingScan rs;
  rs.fromScan2D(*oScan2D);

  EXPECT_TRUE(rs.azimuthSpan > 0) << "Rotation: CCW";
  EXPECT_NEAR(std::abs(rs.azimuthSpan), M_PI, 1e-4);
  EXPECT_EQ(rs.columnCount, 361);
  EXPECT_EQ(rs.rowCount, 1);

  // std::cout << rs.getDescriptionAsTextValue();
}
