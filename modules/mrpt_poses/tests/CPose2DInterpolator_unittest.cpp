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
#include <mrpt/math/wrap2pi.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose2DInterpolator.h>
#include <mrpt/system/datetime.h>

#include <cmath>
#include <filesystem>

template class mrpt::CTraitsTest<mrpt::poses::CPose2DInterpolator>;

TEST(CPose2DInterpolator, interp)
{
  using namespace mrpt::poses;
  using namespace mrpt;  // for 0.0_deg
  using mrpt::DEG2RAD;
  using mrpt::math::TPose2D;

  auto t0 = mrpt::Clock::now();
  using namespace std::chrono_literals;
  auto dt = 100ms;

  CPose2DInterpolator pose_path;

  pose_path.insert(t0, TPose2D(1., 2., 30.0_deg));
  pose_path.insert(t0 + 2 * dt, TPose2D(1. + 3., 2. + 4., DEG2RAD(30.0 + 20.0)));

  TPose2D interp;
  bool valid;
  pose_path.interpolate(t0 + dt, interp, valid);

  EXPECT_TRUE(valid);

  const TPose2D interp_good(1. + 1.5, 2. + 2.0, DEG2RAD(30.0 + 10.0));
  for (unsigned int i = 0; i < interp_good.size(); i++)
  {
    EXPECT_NEAR(interp_good[i], interp[i], 1e-4);
  }
}

TEST(CPose2DInterpolator, AllMethodsAndAccessors)
{
  using namespace mrpt::poses;
  using mrpt::math::TPose2D;
  using namespace std::chrono_literals;

  auto t0 = mrpt::Clock::now();
  auto dt = 100ms;

  CPose2DInterpolator path;
  EXPECT_TRUE(path.empty());

  path.insert(t0, TPose2D(0, 0, 0));
  path.insert(t0 + dt, CPose2D(1, 1, 0.1));
  path.insert(t0 + 2 * dt, TPose2D(2, 2, 0.2));
  path.insert(t0 + 3 * dt, TPose2D(3, 3, 0.3));

  EXPECT_EQ(path.size(), 4u);
  EXPECT_FALSE(path.empty());

  // The documented default method:
  EXPECT_EQ(path.getInterpolationMethod(), imLinearSlerp);

  const TInterpolatorMethod methods[] = {imSpline, imLinear2Neig, imLinear4Neig, imSSLLLL,
                                         imSSLSLL, imLinearSlerp, imSplineSlerp};
  for (auto m : methods)
  {
    path.setInterpolationMethod(m);
    EXPECT_EQ(path.getInterpolationMethod(), m);

    TPose2D interp;
    bool valid = false;
    path.interpolate(t0 + dt + dt / 2, interp, valid);
    EXPECT_TRUE(valid);

    CPose2D interpC;
    bool validC = false;
    path.interpolate(t0 + dt + dt / 2, interpC, validC);
    EXPECT_TRUE(validC);
  }

  mrpt::math::TPoint2D bbMin;
  mrpt::math::TPoint2D bbMax;
  path.getBoundingBox(bbMin, bbMax);
  EXPECT_NEAR(bbMin.x, 0.0, 1e-9);
  EXPECT_NEAR(bbMax.x, 3.0, 1e-9);

  path.filter(0, 3);
  EXPECT_EQ(path.size(), 4u);

  TPose2D prevPose;
  bool foundPrev = path.getPreviousPoseWithMinDistance(t0 + 3 * dt, 1.0, prevPose);
  (void)foundPrev;

  EXPECT_THROW(path.setMaxTimeInterpolation(mrpt::Clock::duration(-1)), std::exception);
  path.setMaxTimeInterpolation(std::chrono::seconds(10));
  EXPECT_GT(path.getMaxTimeInterpolation().count(), 0);

  EXPECT_NE(path.begin(), path.end());
  EXPECT_NE(path.find(t0), path.end());
  EXPECT_NE(path.lower_bound(t0), path.end());
  EXPECT_NE(path.upper_bound(t0), path.end());

  const auto tmpTxt =
      (std::filesystem::temp_directory_path() / "mrpt_test_pose2dinterp.txt").string();
  EXPECT_TRUE(path.saveToTextFile(tmpTxt));

  CPose2DInterpolator loaded;
  EXPECT_TRUE(loaded.loadFromTextFile(tmpTxt));
  EXPECT_EQ(loaded.size(), path.size());

  const auto tmpInterp =
      (std::filesystem::temp_directory_path() / "mrpt_test_pose2dinterp_dense.txt").string();
  EXPECT_TRUE(path.saveInterpolatedToTextFile(tmpInterp, dt));

  auto it = path.begin();
  path.erase(it);
  EXPECT_EQ(path.size(), 3u);

  path.clear();
  EXPECT_TRUE(path.empty());
}

TEST(CPose2DInterpolator, EdgeCasesOutOfRangeAndMaxTimeInterp)
{
  using namespace mrpt::poses;
  using mrpt::math::TPose2D;
  using namespace std::chrono_literals;

  auto t0 = mrpt::Clock::now();
  auto dt = 100ms;

  CPose2DInterpolator path;
  path.insert(t0, TPose2D(0, 0, 0));
  path.insert(t0 + dt, TPose2D(1, 1, 0.1));
  path.insert(t0 + 2 * dt, TPose2D(2, 2, 0.2));
  path.insert(t0 + 3 * dt, TPose2D(3, 3, 0.3));

  TPose2D interp;
  bool valid = true;
  path.interpolate(t0 - dt, interp, valid);
  EXPECT_FALSE(valid);

  valid = true;
  path.interpolate(t0 + 10 * dt, interp, valid);
  EXPECT_FALSE(valid);

  path.setInterpolationMethod(imSpline);
  valid = true;
  path.interpolate(t0 + dt / 2, interp, valid);
  EXPECT_FALSE(valid);

  path.setMaxTimeInterpolation(std::chrono::milliseconds(1));
  path.setInterpolationMethod(imLinearSlerp);
  valid = true;
  path.interpolate(t0 + dt + dt / 2, interp, valid);
  EXPECT_FALSE(valid);

  CPose2D prevPoseC;
  bool foundPrev = path.getPreviousPoseWithMinDistance(t0 + 3 * dt, 1.0, prevPoseC);
  (void)foundPrev;

  // filter() over x,y,phi (component indices valid for TPose2D):
  for (unsigned int comp = 0; comp < 3; comp++)
  {
    CPose2DInterpolator copy = path;
    copy.filter(comp, 3);
    EXPECT_EQ(copy.size(), path.size());
  }
}

TEST(CPose2DInterpolator, slerpMethodsReturnWrappedAngle)
{
  using namespace mrpt::poses;
  using mrpt::math::TPose2D;
  const auto t0 = mrpt::Clock::fromDouble(1000.0);
  const auto dt = std::chrono::milliseconds(1000);
  CPose2DInterpolator path;
  path.insert(t0, TPose2D(0, 0, 3.0));
  path.insert(t0 + dt, TPose2D(0, 0, -3.0));
  path.insert(t0 + 2 * dt, TPose2D(0, 0, -2.9));
  path.insert(t0 + 3 * dt, TPose2D(0, 0, -2.8));
  for (const auto m : {imLinearSlerp, imSplineSlerp})
  {
    path.setInterpolationMethod(m);
    TPose2D out;
    bool valid = false;
    path.interpolate(t0 + dt + dt / 2, out, valid);
    ASSERT_TRUE(valid);
    EXPECT_NEAR(out.phi, -2.95, 1e-9);
  }
  // Linear interpolation across +-pi:
  path.setInterpolationMethod(imLinear2Neig);
  TPose2D out;
  bool valid = false;
  path.interpolate(t0 + dt / 2, out, valid);
  ASSERT_TRUE(valid);
  EXPECT_NEAR(std::abs(out.phi), M_PI, 1e-9);
}

// Interpolated values of all methods for a constant-velocity motion: linear
// translation and a constant heading rate across +-pi, which all methods must
// reproduce exactly.
TEST(CPose2DInterpolator, AllMethodsValuesConstantVelocity)
{
  using namespace mrpt::poses;
  using mrpt::math::TPose2D;

  const auto t0 = mrpt::Clock::fromDouble(1000.0);
  const double dt = 0.1;
  const auto truth = [](double t) { return TPose2D(1.0 + 2.0 * t, -0.5 * t, 3.0 + 0.5 * t); };
  CPose2DInterpolator path;
  for (int i = 0; i < 6; i++)
  {
    path.insert(mrpt::Clock::fromDouble(1000.0 + i * dt), truth(i * dt));
  }
  for (const auto m :
       {imSpline, imLinear2Neig, imLinear4Neig, imSSLLLL, imSSLSLL, imLinearSlerp, imSplineSlerp})
  {
    path.setInterpolationMethod(m);
    for (const double tq : {0.15, 0.25, 0.27, 0.33})
    {
      TPose2D p;
      bool valid = false;
      path.interpolate(t0 + std::chrono::microseconds(static_cast<int64_t>(tq * 1e6)), p, valid);
      ASSERT_TRUE(valid);
      const TPose2D gt = truth(tq);
      EXPECT_NEAR(p.x, gt.x, 1e-6) << "method=" << static_cast<int>(m) << " t=" << tq;
      EXPECT_NEAR(p.y, gt.y, 1e-6);
      EXPECT_NEAR(mrpt::math::wrapToPi(p.phi - gt.phi), 0.0, 1e-6)
          << "method=" << static_cast<int>(m) << " t=" << tq << " phi=" << p.phi;
      EXPECT_GE(p.phi, -M_PI);
      EXPECT_LE(p.phi, M_PI);
    }
  }
}
