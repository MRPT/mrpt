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
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/FrameTransformer.h>

#include <Eigen/Dense>

template <int DIM>
void run_tf_test1(const mrpt::poses::CPose2D& A2B_)
{
  using namespace mrpt::poses;

  FrameTransformer<DIM> tf;

  // Pub:
  const auto real_A2B = static_cast<typename FrameTransformer<DIM>::pose_t>(A2B_);
  tf.sendTransform("A", "B", real_A2B);

  // Read:
  {
    typename FrameTransformer<DIM>::pose_t A2B_looked_up;
    const auto ret = tf.lookupTransform("B", "A", A2B_looked_up);
    EXPECT_EQ(ret, mrpt::poses::LKUP_GOOD);
    EXPECT_NEAR(
        .0, (real_A2B.asVectorVal() - A2B_looked_up.asVectorVal()).array().abs().sum(), 1e-6);
  }
}

TEST(FrameTransformer, SimplePublishAndLookup)
{
  const mrpt::poses::CPose2D test_A2B(5.0, 2.0, 0.3);

  run_tf_test1<2>(test_A2B);
  run_tf_test1<3>(test_A2B);
}

// Chains of transforms, looked up in both directions, against the explicit
// compositions. lookupTransform(target, source) returns the pose of target
// with respect to source.
template <int DIM>
void run_tf_chain_test()
{
  using namespace mrpt::poses;
  using pose_t = typename FrameTransformer<DIM>::pose_t;

  const auto map2odom = pose_t(mrpt::poses::CPose3D(1.0, 2.0, 0.0, 0.5, 0.0, 0.0));
  const auto odom2base = pose_t(mrpt::poses::CPose3D(3.0, -1.0, 0.0, -1.2, 0.0, 0.0));
  const auto base2laser = pose_t(mrpt::poses::CPose3D(0.2, 0.1, 0.0, 3.0, 0.0, 0.0));
  const auto map2gps = pose_t(mrpt::poses::CPose3D(-5.0, 4.0, 0.0, 2.0, 0.0, 0.0));

  FrameTransformer<DIM> tf;
  tf.sendTransform("map", "odom", map2odom);
  tf.sendTransform("odom", "base_link", odom2base);
  tf.sendTransform("base_link", "laser", base2laser);
  tf.sendTransform("map", "gps", map2gps);

  const auto expectLookup =
      [&](const std::string& target, const std::string& source, const pose_t& expected)
  {
    pose_t p;
    EXPECT_EQ(tf.lookupTransform(target, source, p), LKUP_GOOD) << target << " " << source;
    EXPECT_NEAR(.0, (expected.asVectorVal() - p.asVectorVal()).array().abs().maxCoeff(), 1e-12)
        << target << " wrt " << source << ": " << p << " expected: " << expected;
  };

  expectLookup("laser", "map", map2odom + odom2base + base2laser);
  expectLookup("map", "laser", -(map2odom + odom2base + base2laser));
  expectLookup("odom", "base_link", -odom2base);
  expectLookup("gps", "laser", (-(map2odom + odom2base + base2laser)) + map2gps);
  expectLookup("map", "map", pose_t());

  pose_t p;
  EXPECT_EQ(tf.lookupTransform("map", "unknown", p), LKUP_UNKNOWN_FRAME);
  EXPECT_EQ(tf.lookupTransform("unknown", "map", p), LKUP_UNKNOWN_FRAME);
  tf.sendTransform("world", "other", pose_t());
  EXPECT_EQ(tf.lookupTransform("other", "map", p), LKUP_NO_CONNECTIVITY);
}

TEST(FrameTransformer, ChainsAndInverseLookups)
{
  run_tf_chain_test<2>();
  run_tf_chain_test<3>();
}
