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

/** Unit tests for the range-only (beacon) rejection sampler: parameter
 *  loading from a beacon map + observation, the sampling itself, and the
 *  geometry of the drawn poses.
 */

#include <gtest/gtest.h>
#include <mrpt/maps/CLandmarksMap.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/obs/CObservationBeaconRanges.h>
#include <mrpt/random.h>
#include <mrpt/slam/CRejectionSamplingRangeOnlyLocalization.h>

#include <cmath>

using mrpt::slam::CRejectionSamplingRangeOnlyLocalization;

namespace
{
/** A beacon map with the given (id, x, y, z) beacons. */
mrpt::maps::CLandmarksMap make_beacon_map(
    const std::vector<std::tuple<int64_t, double, double, double>>& beacons)
{
  mrpt::maps::CLandmarksMap m;
  for (const auto& [id, x, y, z] : beacons)
  {
    mrpt::maps::CLandmark lm;
    lm.ID = id;
    lm.pose_mean = mrpt::math::TPoint3D(x, y, z);
    m.landmarks.push_back(lm);
  }
  return m;
}

/** An observation of ranges to the given beacon IDs, with the sensor at the
 *  robot origin. */
mrpt::obs::CObservationBeaconRanges make_obs(
    const std::vector<std::pair<int64_t, float>>& idAndRange)
{
  mrpt::obs::CObservationBeaconRanges o;
  for (const auto& [id, range] : idAndRange)
  {
    mrpt::obs::CObservationBeaconRanges::TMeasurement m;
    m.beaconID = id;
    m.sensedDistance = range;
    m.sensorLocationOnRobot = mrpt::poses::CPoint3D(0, 0, 0);
    o.sensedData.push_back(m);
  }
  return o;
}
}  // namespace

TEST(CRejectionSamplingRangeOnly, setParams_accepts_a_matching_beacon)
{
  auto map = make_beacon_map({
      {1, 5.0, .0, .0}
  });
  auto obs = make_obs({
      {1, 3.0f}
  });

  CRejectionSamplingRangeOnlyLocalization rs;
  EXPECT_TRUE(rs.setParams(map, obs, 0.2f, mrpt::poses::CPose2D(0, 0, 0)));
}

TEST(CRejectionSamplingRangeOnly, setParams_rejects_an_observation_of_unknown_beacons)
{
  auto map = make_beacon_map({
      {1, 5.0, .0, .0}
  });
  auto obs = make_obs({
      {99 /*not in the map*/, 3.0f}
  });

  CRejectionSamplingRangeOnlyLocalization rs;
  EXPECT_FALSE(rs.setParams(map, obs, 0.2f, mrpt::poses::CPose2D(0, 0, 0)));
}

TEST(CRejectionSamplingRangeOnly, setParams_rejects_an_empty_observation)
{
  auto map = make_beacon_map({
      {1, 5.0, .0, .0}
  });
  auto obs = make_obs({});

  CRejectionSamplingRangeOnlyLocalization rs;
  EXPECT_FALSE(rs.setParams(map, obs, 0.2f, mrpt::poses::CPose2D(0, 0, 0)));
}

TEST(CRejectionSamplingRangeOnly, a_range_shorter_than_the_height_difference_is_dropped)
{
  // Beacon 4 m above the robot plane, but only 1 m of measured range: the
  // circle of possible positions at the robot's height does not exist.
  auto map = make_beacon_map({
      {1, 5.0, .0, 4.0}
  });
  auto obs = make_obs({
      {1, 1.0f}
  });

  CRejectionSamplingRangeOnlyLocalization rs;
  EXPECT_FALSE(rs.setParams(map, obs, 0.2f, mrpt::poses::CPose2D(0, 0, 0), 0.0f));
}

TEST(CRejectionSamplingRangeOnly, the_beacon_height_is_projected_onto_the_robot_plane)
{
  // 5 m of slant range to a beacon 3 m up => 4 m radius at the robot plane.
  auto map = make_beacon_map({
      {1, .0, .0, 3.0}
  });
  auto obs = make_obs({
      {1, 5.0f}
  });

  CRejectionSamplingRangeOnlyLocalization rs;
  ASSERT_TRUE(rs.setParams(map, obs, 0.05f, mrpt::poses::CPose2D(0, 0, 0), 0.0f));

  mrpt::random::getRandomGenerator().randomize(1234);
  const auto samples = rs.rejectionSampling(50);
  ASSERT_EQ(samples.size(), 50U);

  for (const auto& s : samples)
  {
    const double r = std::hypot(s.d->x(), s.d->y());
    EXPECT_NEAR(r, 4.0, 0.5) << "sample at " << s.d->asString();
  }
}

TEST(CRejectionSamplingRangeOnly, samples_lie_near_the_intersection_of_two_range_circles)
{
  // Two beacons 6 m apart, both 5 m away => the robot is near (3, +-4).
  auto map = make_beacon_map({
      {1,  .0, .0, .0},
      {2, 6.0, .0, .0}
  });
  auto obs = make_obs({
      {1, 5.0f},
      {2, 5.0f}
  });

  CRejectionSamplingRangeOnlyLocalization rs;
  ASSERT_TRUE(rs.setParams(map, obs, 0.1f, mrpt::poses::CPose2D(3, 4, 0)));

  mrpt::random::getRandomGenerator().randomize(4321);
  const auto samples = rs.rejectionSampling(40);
  ASSERT_EQ(samples.size(), 40U);

  for (const auto& s : samples)
  {
    const double d1 = std::hypot(s.d->x(), s.d->y());
    const double d2 = std::hypot(s.d->x() - 6.0, s.d->y());
    // Every accepted sample must be consistent with *both* ranges:
    EXPECT_NEAR(d1, 5.0, 1.0) << s.d->asString();
    EXPECT_NEAR(d2, 5.0, 1.0) << s.d->asString();
  }
}

TEST(CRejectionSamplingRangeOnly, sampling_without_setParams_throws)
{
  CRejectionSamplingRangeOnlyLocalization rs;
  EXPECT_ANY_THROW((void)rs.rejectionSampling(1));
}

TEST(CRejectionSamplingRangeOnly, the_sensor_offset_on_the_robot_is_compensated)
{
  // The sensor sits 1 m ahead of the robot origin, so the robot pose must be
  // that much "behind" the circle the sensor lies on.
  auto map = make_beacon_map({
      {1, .0, .0, .0}
  });

  mrpt::obs::CObservationBeaconRanges o;
  {
    mrpt::obs::CObservationBeaconRanges::TMeasurement m;
    m.beaconID = 1;
    m.sensedDistance = 4.0f;
    m.sensorLocationOnRobot = mrpt::poses::CPoint3D(1.0, 0, 0);
    o.sensedData.push_back(m);
  }

  CRejectionSamplingRangeOnlyLocalization rs;
  ASSERT_TRUE(rs.setParams(map, o, 0.05f, mrpt::poses::CPose2D(0, 0, 0)));

  mrpt::random::getRandomGenerator().randomize(99);
  const auto samples = rs.rejectionSampling(30);
  ASSERT_EQ(samples.size(), 30U);

  for (const auto& s : samples)
  {
    // Re-project the sensor from the sampled robot pose: it must sit on the
    // 4 m circle around the beacon.
    const mrpt::math::TPoint2D sensor =
        mrpt::math::TPose2D(s.d->x(), s.d->y(), s.d->phi()) + mrpt::math::TPoint2D(1.0, 0);
    EXPECT_NEAR(std::hypot(sensor.x, sensor.y), 4.0, 0.5) << s.d->asString();
  }
}

TEST(CRejectionSamplingRangeOnly, angle_prefiltering_can_be_disabled)
{
  auto map = make_beacon_map({
      {1,  .0, .0, .0},
      {2, 6.0, .0, .0}
  });
  auto obs = make_obs({
      {1, 5.0f},
      {2, 5.0f}
  });

  CRejectionSamplingRangeOnlyLocalization rs;
  ASSERT_TRUE(rs.setParams(
      map, obs, 0.1f, mrpt::poses::CPose2D(3, 4, 0), 0.0f, false /*autoCheckAngleRanges*/));

  mrpt::random::getRandomGenerator().randomize(7);
  const auto samples = rs.rejectionSampling(10);
  EXPECT_EQ(samples.size(), 10U);
}

TEST(CRejectionSamplingRangeOnly, a_non_zero_robot_height_is_honored)
{
  // Beacon at z=3, robot at z=1 => 2 m of height difference; a 2.5 m slant
  // range leaves a 1.5 m radius at the robot plane.
  auto map = make_beacon_map({
      {1, .0, .0, 3.0}
  });
  auto obs = make_obs({
      {1, 2.5f}
  });

  CRejectionSamplingRangeOnlyLocalization rs;
  ASSERT_TRUE(rs.setParams(map, obs, 0.05f, mrpt::poses::CPose2D(0, 0, 0), 1.0f /*robot_z*/));

  mrpt::random::getRandomGenerator().randomize(11);
  const auto samples = rs.rejectionSampling(20);
  ASSERT_EQ(samples.size(), 20U);
  for (const auto& s : samples)
  {
    EXPECT_NEAR(std::hypot(s.d->x(), s.d->y()), 1.5, 0.4) << s.d->asString();
  }
}
