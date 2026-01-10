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

#include <CTraitsTest.h>
#include <gtest/gtest.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DInterpolator.h>
#include <mrpt/system/datetime.h>

#include <Eigen/Dense>

template class mrpt::CTraitsTest<mrpt::poses::CPose3DInterpolator>;

TEST(CPose3DInterpolator, interp)
{
  using namespace mrpt::poses;
  using namespace mrpt;  // for 0.0_deg
  using mrpt::DEG2RAD;
  using mrpt::math::CMatrixDouble44;
  using mrpt::math::TPose3D;

  auto t0 = mrpt::Clock::now();
  mrpt::Clock::duration dt(std::chrono::milliseconds(100));

  CPose3DInterpolator pose_path;

  pose_path.insert(t0, TPose3D(1., 2., 3., 30.0_deg, .0_deg, .0_deg));
  pose_path.insert(
      t0 + 2 * dt, TPose3D(1. + 3., 2. + 4., 3. + 5., DEG2RAD(30.0 + 20.0), .0_deg, .0_deg));

  TPose3D interp;
  bool valid;
  pose_path.interpolate(t0 + dt, interp, valid);

  EXPECT_TRUE(valid);
  const TPose3D interp_good(1. + 1.5, 2. + 2.0, 3. + 2.5, DEG2RAD(30.0 + 10.0), .0_deg, .0_deg);
  EXPECT_NEAR(
      .0,
      (CPose3D(interp_good).getHomogeneousMatrixVal<CMatrixDouble44>() -
       CPose3D(interp).getHomogeneousMatrixVal<CMatrixDouble44>())
          .array()
          .abs()
          .sum(),
      2e-4);
}
