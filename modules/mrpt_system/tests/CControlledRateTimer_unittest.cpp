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

/** Tests for CControlledRateTimer. The assertions on the rate-estimator are
 *  written to be independent of how fast the machine actually runs: they
 *  compare the filtered estimate against the raw one rather than against any
 *  absolute timing.
 */

#include <gtest/gtest.h>
#include <mrpt/system/CControlledRateTimer.h>

#include <cmath>

using mrpt::system::CControlledRateTimer;

TEST(CControlledRateTimer, documented_default_parameters)
{
  // These must agree with the "[default=...]" notes in the header docs.
  CControlledRateTimer t;
  EXPECT_NEAR(t.controllerParam_Kp(), 1.0, 1e-12);
  EXPECT_NEAR(t.controllerParam_Ti(), 0.1, 1e-12);
  EXPECT_NEAR(t.lowPassParam_a0(), 0.99, 1e-12);
  EXPECT_NEAR(t.followErrorRatioToRaiseWarning(), 0.20, 1e-12);
}

TEST(CControlledRateTimer, parameter_validation)
{
  CControlledRateTimer t;

  EXPECT_ANY_THROW(t.controllerParam_Kp(.0));
  EXPECT_ANY_THROW(t.controllerParam_Kp(-1.0));
  EXPECT_NO_THROW(t.controllerParam_Kp(2.0));
  EXPECT_NEAR(t.controllerParam_Kp(), 2.0, 1e-12);

  // Ti may be zero (pure proportional), but not negative:
  EXPECT_ANY_THROW(t.controllerParam_Ti(-1e-6));
  EXPECT_NO_THROW(t.controllerParam_Ti(.0));

  // a0 must lie in (0, 1]:
  EXPECT_ANY_THROW(t.lowPassParam_a0(.0));
  EXPECT_ANY_THROW(t.lowPassParam_a0(1.5));
  EXPECT_NO_THROW(t.lowPassParam_a0(1.0));
  EXPECT_NEAR(t.lowPassParam_a0(), 1.0, 1e-12);

  EXPECT_ANY_THROW(t.followErrorRatioToRaiseWarning(.0));
  EXPECT_ANY_THROW(t.followErrorRatioToRaiseWarning(1.5));
  EXPECT_NO_THROW(t.followErrorRatioToRaiseWarning(0.5));
}

TEST(CControlledRateTimer, setRate_validation_and_readback)
{
  CControlledRateTimer t(10.0);
  EXPECT_ANY_THROW(t.setRate(.0));
  EXPECT_ANY_THROW(t.setRate(-5.0));

  // Re-setting the same rate is a no-op and must not throw:
  EXPECT_NO_THROW(t.setRate(10.0));

  EXPECT_NO_THROW(t.setRate(50.0));
  // Before any sleep() the controller output starts at the set-point:
  EXPECT_NEAR(t.estimatedRate(), 50.0, 1e-9);
}

TEST(CControlledRateTimer, a0_of_one_freezes_the_estimate_on_the_setpoint)
{
  // estimation = a0*former_estimation + (1-a0)*input, so a0==1 must ignore
  // the measured rate entirely. This is the assertion that pins the header
  // documentation to the implementation.
  CControlledRateTimer t(200.0);
  t.lowPassParam_a0(1.0);
  t.setMinLoggingLevel(mrpt::system::LVL_ERROR);

  for (int i = 0; i < 5; i++)
  {
    t.sleep();
  }

  EXPECT_NEAR(t.estimatedRate(), 200.0, 1e-9);
  // ... while the unfiltered measurement did move away from it:
  EXPECT_GT(t.estimatedRateRaw(), .0);
}

// Note: a "larger a0 keeps the estimate nearer the set-point" test would be
// flaky. That only holds while the achieved rate is far from the set-point;
// on a machine that hits the target accurately the raw rate *is* the
// set-point, both estimates converge on it, and the comparison is decided by
// noise. The two tests here pin the filter's weighting without depending on
// the rate actually achieved.
TEST(CControlledRateTimer, a_small_a0_tracks_the_raw_measurement)
{
  // The complement of the test above: weighting the former estimation by
  // almost nothing makes the estimate follow the raw rate closely.
  CControlledRateTimer t(200.0);
  t.lowPassParam_a0(1e-6);
  t.setMinLoggingLevel(mrpt::system::LVL_ERROR);

  for (int i = 0; i < 5; i++)
  {
    t.sleep();
  }

  ASSERT_GT(t.estimatedRateRaw(), .0);
  const double relErr = std::abs(t.estimatedRate() - t.estimatedRateRaw()) / t.estimatedRateRaw();
  EXPECT_LT(relErr, 1e-3);
}

TEST(CControlledRateTimer, sleep_runs_and_reports_a_rate)
{
  CControlledRateTimer t(500.0);
  t.setMinLoggingLevel(mrpt::system::LVL_ERROR);

  for (int i = 0; i < 3; i++)
  {
    t.sleep();
  }

  EXPECT_GT(t.actualControlledRate(), .0);
  EXPECT_GT(t.estimatedRate(), .0);
  EXPECT_TRUE(std::isfinite(t.estimatedRate()));
  EXPECT_TRUE(std::isfinite(t.actualControlledRate()));
}

TEST(CControlledRateTimer, changing_the_rate_resets_the_estimate)
{
  CControlledRateTimer t(100.0);
  t.setMinLoggingLevel(mrpt::system::LVL_ERROR);
  t.sleep();
  t.sleep();

  t.setRate(400.0);
  // setRate() re-seeds the estimator with the new set-point:
  EXPECT_NEAR(t.estimatedRate(), 400.0, 1e-9);
}
