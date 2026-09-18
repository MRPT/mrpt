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
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/carmen_log_tools.h>

#include <sstream>

using namespace mrpt::obs;

TEST(carmen_log_tools, ParseRobotLaser)
{
  // Note: the CARMEN ROBOTLASER format has no separate sensor-name field;
  // the "ROBOTLASER" keyword token itself ends up as CObservation::sensorLabel.
  std::istringstream ss(
      "ROBOTLASER 0 0.0 3.14159 0.01 80.0 0.0 0 3 1.0 2.0 3.0 0 "
      "0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 1000000 robot1\n");

  std::vector<CObservation::Ptr> obs;
  const bool ok = carmen_log_parse_line(ss, obs, mrpt::Clock::now());
  EXPECT_TRUE(ok);
  ASSERT_EQ(obs.size(), 2u);

  auto odo = std::dynamic_pointer_cast<CObservationOdometry>(obs[0]);
  ASSERT_TRUE(odo);
  EXPECT_EQ(odo->sensorLabel, "ODOMETRY");

  auto laser = std::dynamic_pointer_cast<CObservation2DRangeScan>(obs[1]);
  ASSERT_TRUE(laser);
  EXPECT_EQ(laser->sensorLabel, "ROBOTLASER");
  EXPECT_EQ(laser->getScanSize(), 3u);
  EXPECT_NEAR(laser->getScanRange(0), 1.0f, 1e-4f);
  EXPECT_NEAR(laser->getScanRange(2), 3.0f, 1e-4f);
}

TEST(carmen_log_tools, ParseFLaser)
{
  // Default "robot_front_laser_max" is 81.0, so use a range far beyond it to
  // exercise the "invalid range" branch.
  std::istringstream ss("FLASER 2 1.0 999.0 0.0 0.0 0.0 0.0 0.0 0.0 1000000 robot1\n");

  std::vector<CObservation::Ptr> obs;
  const bool ok = carmen_log_parse_line(ss, obs, mrpt::Clock::now());
  EXPECT_TRUE(ok);
  ASSERT_EQ(obs.size(), 2u);

  auto laser = std::dynamic_pointer_cast<CObservation2DRangeScan>(obs[1]);
  ASSERT_TRUE(laser);
  EXPECT_EQ(laser->getScanSize(), 2u);
  // out-of-range value should be marked as invalid:
  EXPECT_FALSE(laser->getScanRangeValidity(1));
}

TEST(carmen_log_tools, ParseRLaser)
{
  std::istringstream ss("RLASER 1 0.5 0.0 0.0 0.0 0.0 0.0 1000000 robot1\n");

  std::vector<CObservation::Ptr> obs;
  const bool ok = carmen_log_parse_line(ss, obs, mrpt::Clock::now());
  EXPECT_TRUE(ok);
  ASSERT_EQ(obs.size(), 2u);
}

TEST(carmen_log_tools, ParseParamThenFLaser)
{
  std::istringstream ss(
      "PARAM robot_front_laser_max 50.0\n"
      "FLASER 1 1.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 1000000 robot1\n");

  std::vector<CObservation::Ptr> obs;
  // First line: a PARAM entry, no observations returned.
  bool ok = carmen_log_parse_line(ss, obs, mrpt::Clock::now());
  EXPECT_TRUE(ok);
  EXPECT_TRUE(obs.empty());

  // Second line: FLASER, using the previously-set maxRange param.
  ok = carmen_log_parse_line(ss, obs, mrpt::Clock::now());
  EXPECT_TRUE(ok);
  ASSERT_EQ(obs.size(), 2u);
  auto laser = std::dynamic_pointer_cast<CObservation2DRangeScan>(obs[1]);
  ASSERT_TRUE(laser);
  EXPECT_NEAR(laser->maxRange, 50.0f, 1e-3f);
}

TEST(carmen_log_tools, EndOfFileReturnsFalse)
{
  std::istringstream ss("");
  std::vector<CObservation::Ptr> obs;
  const bool ok = carmen_log_parse_line(ss, obs, mrpt::Clock::now());
  EXPECT_FALSE(ok);
}

TEST(carmen_log_tools, MalformedLineThrows)
{
  std::istringstream ss("ROBOTLASER not_enough_fields\n");
  std::vector<CObservation::Ptr> obs;
  EXPECT_THROW(carmen_log_parse_line(ss, obs, mrpt::Clock::now()), std::exception);
}

TEST(carmen_log_tools, UnknownLineIgnored)
{
  std::istringstream ss("ODOM 1.0 2.0 3.0\n");
  std::vector<CObservation::Ptr> obs;
  const bool ok = carmen_log_parse_line(ss, obs, mrpt::Clock::now());
  EXPECT_TRUE(ok);
  EXPECT_TRUE(obs.empty());
}
