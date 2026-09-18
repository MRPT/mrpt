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
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/graphslam/misc/TSlidingWindow.h>

#include <cmath>
#include <sstream>

using mrpt::graphslam::TSlidingWindow;

TEST(TSlidingWindow, default_state)
{
  TSlidingWindow w;
  EXPECT_EQ(w.getWindowSize(), 5U);
  EXPECT_FALSE(w.windowIsFull());
}

TEST(TSlidingWindow, empty_window_returns_zero_statistics)
{
  TSlidingWindow w;
  // No measurements yet: every statistic must be a well-defined 0, never NaN.
  EXPECT_EQ(w.getMedian(), .0);
  EXPECT_TRUE(std::isfinite(w.getMean()));
  EXPECT_EQ(w.getMean(), .0);
  EXPECT_TRUE(std::isfinite(w.getStdDev()));
  EXPECT_EQ(w.getStdDev(), .0);
}

TEST(TSlidingWindow, empty_window_comparisons_are_well_defined)
{
  TSlidingWindow w;
  // Against a NaN mean every comparison is false, so "above" would report
  // false for *any* input, however large.
  EXPECT_TRUE(w.evaluateMeasurementAbove(1.0));
  EXPECT_FALSE(w.evaluateMeasurementBelow(1.0));
  EXPECT_FALSE(w.evaluateMeasurementAbove(-1.0));
  EXPECT_TRUE(w.evaluateMeasurementBelow(-1.0));
  // The gaussian band is degenerate with no data (mean and sigma are both 0),
  // so it accepts nothing -- but for a defined reason, not because every
  // comparison against a NaN is false.
  EXPECT_FALSE(w.evaluateMeasurementInGaussian(.0));
}

TEST(TSlidingWindow, mean_and_median_of_a_partially_filled_window)
{
  TSlidingWindow w;
  w.resizeWindow(5);
  for (double v : {1.0, 2.0, 3.0})
  {
    w.addNewMeasurement(v);
  }
  EXPECT_FALSE(w.windowIsFull());
  EXPECT_NEAR(w.getMean(), 2.0, 1e-12);
  EXPECT_NEAR(w.getMedian(), 2.0, 1e-12);
}

TEST(TSlidingWindow, std_dev_uses_the_measurement_count_not_the_window_size)
{
  TSlidingWindow w;
  w.resizeWindow(100);  // much larger than the number of measurements
  for (double v : {2.0, 4.0, 4.0, 4.0, 5.0, 5.0, 7.0, 9.0})
  {
    w.addNewMeasurement(v);
  }
  // Population std dev of the 8 samples above is exactly 2.0. Dividing by the
  // window size (100) instead would report ~0.57.
  EXPECT_NEAR(w.getMean(), 5.0, 1e-12);
  EXPECT_NEAR(w.getStdDev(), 2.0, 1e-9);
}

TEST(TSlidingWindow, oldest_measurements_are_dropped_once_full)
{
  TSlidingWindow w;
  w.resizeWindow(3);
  w.addNewMeasurement(1.0);
  w.addNewMeasurement(2.0);
  w.addNewMeasurement(3.0);
  EXPECT_TRUE(w.windowIsFull());
  EXPECT_NEAR(w.getMean(), 2.0, 1e-12);

  w.addNewMeasurement(4.0);  // pushes 1.0 out
  EXPECT_TRUE(w.windowIsFull());
  EXPECT_NEAR(w.getMean(), 3.0, 1e-12);
  EXPECT_NEAR(w.getMedian(), 3.0, 1e-12);
}

TEST(TSlidingWindow, statistics_are_recomputed_after_a_new_measurement)
{
  TSlidingWindow w;
  w.resizeWindow(4);
  w.addNewMeasurement(10.0);
  const double m1 = w.getMean();  // caches the value
  w.addNewMeasurement(20.0);
  EXPECT_NE(w.getMean(), m1);
  EXPECT_NEAR(w.getMean(), 15.0, 1e-12);
}

TEST(TSlidingWindow, shrinking_the_window_drops_the_oldest_measurements)
{
  TSlidingWindow w;
  w.resizeWindow(5);
  for (double v : {1.0, 2.0, 3.0, 4.0, 5.0})
  {
    w.addNewMeasurement(v);
  }
  ASSERT_NEAR(w.getMean(), 3.0, 1e-12);

  w.resizeWindow(2);  // keeps the two most recent: 4 and 5
  EXPECT_EQ(w.getWindowSize(), 2U);
  EXPECT_TRUE(w.windowIsFull());
  EXPECT_NEAR(w.getMean(), 4.5, 1e-12);
}

TEST(TSlidingWindow, resizing_invalidates_the_cached_std_dev)
{
  TSlidingWindow w;
  w.resizeWindow(5);
  for (double v : {1.0, 2.0, 3.0, 4.0, 5.0})
  {
    w.addNewMeasurement(v);
  }
  const double sd_before = w.getStdDev();  // caches it
  ASSERT_GT(sd_before, .0);

  w.resizeWindow(2);  // now only {4, 5} remain: std dev must be 0.5
  EXPECT_NEAR(w.getStdDev(), 0.5, 1e-9);
}

TEST(TSlidingWindow, growing_the_window_keeps_the_measurements)
{
  TSlidingWindow w;
  w.resizeWindow(2);
  w.addNewMeasurement(1.0);
  w.addNewMeasurement(3.0);
  ASSERT_TRUE(w.windowIsFull());

  w.resizeWindow(10);
  EXPECT_EQ(w.getWindowSize(), 10U);
  EXPECT_FALSE(w.windowIsFull());
  EXPECT_NEAR(w.getMean(), 2.0, 1e-12);
  EXPECT_NEAR(w.getStdDev(), 1.0, 1e-9);
}

TEST(TSlidingWindow, gaussian_acceptance_band)
{
  TSlidingWindow w;
  w.resizeWindow(5);
  for (double v : {9.0, 10.0, 10.0, 10.0, 11.0})
  {
    w.addNewMeasurement(v);
  }
  ASSERT_NEAR(w.getMean(), 10.0, 1e-12);

  EXPECT_TRUE(w.evaluateMeasurementInGaussian(10.0));
  EXPECT_FALSE(w.evaluateMeasurementInGaussian(1000.0));
  EXPECT_FALSE(w.evaluateMeasurementInGaussian(-1000.0));
}

TEST(TSlidingWindow, above_and_below_are_complementary)
{
  TSlidingWindow w;
  w.resizeWindow(3);
  w.addNewMeasurement(1.0);
  w.addNewMeasurement(3.0);
  ASSERT_NEAR(w.getMean(), 2.0, 1e-12);

  EXPECT_TRUE(w.evaluateMeasurementAbove(2.5));
  EXPECT_FALSE(w.evaluateMeasurementBelow(2.5));

  EXPECT_FALSE(w.evaluateMeasurementAbove(1.5));
  EXPECT_TRUE(w.evaluateMeasurementBelow(1.5));

  // Exactly at the mean counts as "below" (the check is `> mean`):
  EXPECT_FALSE(w.evaluateMeasurementAbove(2.0));
  EXPECT_TRUE(w.evaluateMeasurementBelow(2.0));
}

TEST(TSlidingWindow, config_file_sets_the_window_size)
{
  mrpt::config::CConfigFileMemory cfg;
  cfg.write("S", "sliding_win_size", 7);

  TSlidingWindow w;
  w.loadFromConfigFile(cfg, "S");
  EXPECT_EQ(w.getWindowSize(), 7U);

  // The key is optional and defaults to 10:
  mrpt::config::CConfigFileMemory empty;
  empty.write("S", "dummy", 0);
  TSlidingWindow w2;
  w2.loadFromConfigFile(empty, "S");
  EXPECT_EQ(w2.getWindowSize(), 10U);
}

TEST(TSlidingWindow, dumpToTextStream_reports_the_contents)
{
  TSlidingWindow w("my_window");
  w.addNewMeasurement(1.5);
  w.addNewMeasurement(2.5);

  std::ostringstream ss;
  w.dumpToTextStream(ss);
  const std::string s = ss.str();

  EXPECT_NE(s.find("my_window"), std::string::npos);
  EXPECT_NE(s.find("1.50"), std::string::npos);
  EXPECT_NE(s.find("2.50"), std::string::npos);
  EXPECT_NE(s.find("m_win_size"), std::string::npos);
}
