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
#include <mrpt/poses/Lie/SO.h>
#include <mrpt/system/datetime.h>

#include <Eigen/Dense>
#include <filesystem>
#include <fstream>

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

TEST(CPose3DInterpolator, AllMethodsAndAccessors)
{
  using namespace mrpt::poses;
  using mrpt::math::TPose3D;

  auto t0 = mrpt::Clock::now();
  mrpt::Clock::duration dt(std::chrono::milliseconds(100));

  CPose3DInterpolator path;
  EXPECT_TRUE(path.empty());

  path.insert(t0, TPose3D(0, 0, 0, 0, 0, 0));
  path.insert(t0 + dt, CPose3D(1, 1, 1, 0.1, 0.05, 0.02));
  path.insert(t0 + 2 * dt, TPose3D(2, 2, 2, 0.2, 0.1, 0.04));
  path.insert(t0 + 3 * dt, TPose3D(3, 3, 3, 0.3, 0.15, 0.06));

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

    TPose3D interp;
    bool valid = false;
    path.interpolate(t0 + dt + dt / 2, interp, valid);
    EXPECT_TRUE(valid);

    CPose3D interpC;
    bool validC = false;
    path.interpolate(t0 + dt + dt / 2, interpC, validC);
    EXPECT_TRUE(validC);
  }

  // Bounding box:
  mrpt::math::TPoint3D bbMin;
  mrpt::math::TPoint3D bbMax;
  path.getBoundingBox(bbMin, bbMax);
  EXPECT_NEAR(bbMin.x, 0.0, 1e-9);
  EXPECT_NEAR(bbMax.x, 3.0, 1e-9);

  // Filter (smoothing):
  path.filter(0, 3);
  EXPECT_EQ(path.size(), 4u);

  // getPreviousPoseWithMinDistance:
  TPose3D prevPose;
  bool foundPrev = path.getPreviousPoseWithMinDistance(t0 + 3 * dt, 1.0, prevPose);
  (void)foundPrev;

  // Max time interpolation:
  EXPECT_THROW(path.setMaxTimeInterpolation(mrpt::Clock::duration(-1)), std::exception);
  path.setMaxTimeInterpolation(std::chrono::seconds(10));
  EXPECT_GT(path.getMaxTimeInterpolation().count(), 0);

  // Iteration & lookup:
  EXPECT_NE(path.begin(), path.end());
  EXPECT_NE(path.find(t0), path.end());
  EXPECT_NE(path.lower_bound(t0), path.end());
  EXPECT_NE(path.upper_bound(t0), path.end());
  size_t cnt = 0;
  for (auto it = path.begin(); it != path.end(); ++it)
  {
    cnt++;
  }
  EXPECT_EQ(cnt, path.size());
  cnt = 0;
  for (auto it = path.rbegin(); it != path.rend(); ++it)
  {
    cnt++;
  }
  EXPECT_EQ(cnt, path.size());

  // Save/load round trip:
  const auto tmpTxt =
      (std::filesystem::temp_directory_path() / "mrpt_test_pose3dinterp.txt").string();
  EXPECT_TRUE(path.saveToTextFile(tmpTxt));

  CPose3DInterpolator loaded;
  EXPECT_TRUE(loaded.loadFromTextFile(tmpTxt));
  EXPECT_EQ(loaded.size(), path.size());

  const auto tmpTum =
      (std::filesystem::temp_directory_path() / "mrpt_test_pose3dinterp_tum.txt").string();
  EXPECT_TRUE(path.saveToTextFile_TUM(tmpTum));
  CPose3DInterpolator loadedTum;
  EXPECT_TRUE(loadedTum.loadFromTextFile_TUM(tmpTum));
  EXPECT_EQ(loadedTum.size(), path.size());

  const auto tmpInterp =
      (std::filesystem::temp_directory_path() / "mrpt_test_pose3dinterp_dense.txt").string();
  EXPECT_TRUE(path.saveInterpolatedToTextFile(tmpInterp, dt));

  // erase:
  auto it = path.begin();
  path.erase(it);
  EXPECT_EQ(path.size(), 3u);

  path.clear();
  EXPECT_TRUE(path.empty());
}

TEST(CPose3DInterpolator, loadFromTextFile_TUM_CommentsAndNonFinite)
{
  using namespace mrpt::poses;

  const auto tmpTum =
      (std::filesystem::temp_directory_path() / "mrpt_test_pose3dinterp_tum_edge.txt").string();
  {
    std::ofstream f(tmpTum);
    f << "#timestamp tx ty tz qx qy qz qw\n"
         "   # indented comment\n"
         "\n"
         "  \t \r\n"
         "10.0  0 0 0  0 0 0 1\n"
         "10.1 nan 0 0  0 0 0 1\n"
         "10.2  1 2 3  0 0 0 1\r\n";
  }

  CPose3DInterpolator path;
  EXPECT_TRUE(path.loadFromTextFile_TUM(tmpTum));
  ASSERT_EQ(path.size(), 2u);
  EXPECT_NEAR(path.rbegin()->second.z, 3.0, 1e-9);

  // A file whose rows are all non-finite yields no poses, and fails:
  {
    std::ofstream f(tmpTum);
    f << "10.0 nan 0 0  0 0 0 1\n"
         "10.1 0 inf 0  0 0 0 1\n";
  }
  CPose3DInterpolator allBad;
  EXPECT_FALSE(allBad.loadFromTextFile_TUM(tmpTum));
  EXPECT_TRUE(allBad.empty());
}

TEST(CPose3DInterpolator, EdgeCasesOutOfRangeAndMaxTimeInterp)
{
  using namespace mrpt::poses;
  using mrpt::math::TPose3D;

  auto t0 = mrpt::Clock::now();
  mrpt::Clock::duration dt(std::chrono::milliseconds(100));

  CPose3DInterpolator path;
  path.insert(t0, TPose3D(0, 0, 0, 0, 0, 0));
  path.insert(t0 + dt, TPose3D(1, 1, 1, 0.1, 0.05, 0.02));
  path.insert(t0 + 2 * dt, TPose3D(2, 2, 2, 0.2, 0.1, 0.04));
  path.insert(t0 + 3 * dt, TPose3D(3, 3, 3, 0.3, 0.15, 0.06));

  // Query before the very first point and after the very last one:
  TPose3D interp;
  bool valid = true;
  path.interpolate(t0 - dt, interp, valid);
  EXPECT_FALSE(valid);

  valid = true;
  path.interpolate(t0 + 10 * dt, interp, valid);
  EXPECT_FALSE(valid);

  // A 4-point method queried right at the edges of the path (only 2-3
  // neighbors available on one side):
  path.setInterpolationMethod(imSpline);
  valid = true;
  path.interpolate(t0 + dt / 2, interp, valid);
  EXPECT_FALSE(valid);  // not enough points before t0

  valid = true;
  path.interpolate(t0 + dt + dt / 2, interp, valid);
  EXPECT_TRUE(valid);  // enough points on both sides (2 before, 2 after)

  // maxTimeInterpolation exceeded:
  path.setMaxTimeInterpolation(std::chrono::milliseconds(1));
  path.setInterpolationMethod(imLinearSlerp);
  valid = true;
  path.interpolate(t0 + dt + dt / 2, interp, valid);
  EXPECT_FALSE(valid);

  // getPreviousPoseWithMinDistance, CPose3D overload:
  CPose3D prevPoseC;
  bool foundPrev = path.getPreviousPoseWithMinDistance(t0 + 3 * dt, 1.0, prevPoseC);
  (void)foundPrev;

  // filter() over every pose component (x,y,z,yaw,pitch,roll):
  for (unsigned int comp = 0; comp < 6; comp++)
  {
    CPose3DInterpolator copy = path;
    copy.filter(comp, 3);
    EXPECT_EQ(copy.size(), path.size());
  }
}

TEST(CPose3DInterpolator, splineSlerpNeedsFourPoints)
{
  using namespace mrpt::poses;
  using mrpt::math::TPose3D;
  const auto t0 = mrpt::Clock::fromDouble(1000.0);
  const auto dt = std::chrono::milliseconds(1000);
  CPose3DInterpolator path;
  path.insert(t0, TPose3D(5, 0, 0, 0, 0, 0));
  path.insert(t0 + dt, TPose3D(6, 0, 0, 0, 0, 0));
  path.insert(t0 + 2 * dt, TPose3D(7, 0, 0, 0, 0, 0));
  path.insert(t0 + 3 * dt, TPose3D(8, 0, 0, 0, 0, 0));
  path.setInterpolationMethod(imSplineSlerp);
  TPose3D out;
  bool valid = false;
  path.interpolate(t0 + dt + dt / 2, out, valid);
  EXPECT_TRUE(valid);
  EXPECT_NEAR(out.x, 6.5, 1e-6);
  // The last segment lacks a fourth point:
  valid = true;
  EXPECT_NO_THROW(path.interpolate(t0 + 2 * dt + dt / 2, out, valid));
  EXPECT_FALSE(valid);
}

TEST(CPose3DInterpolator, linearSlerpAcrossPi)
{
  using namespace mrpt::poses;
  using mrpt::math::TPose3D;
  const auto t0 = mrpt::Clock::fromDouble(1000.0);
  const auto dt = std::chrono::milliseconds(1000);
  CPose3DInterpolator path;
  path.insert(t0, TPose3D(0, 0, 0, 3.0, 0.1, 0.2));
  path.insert(t0 + dt, TPose3D(1, 0, 0, -3.0, 0.1, 0.2));
  path.setInterpolationMethod(imLinearSlerp);
  TPose3D out;
  bool valid = false;
  path.interpolate(t0 + dt / 2, out, valid);
  ASSERT_TRUE(valid);
  // Takes the short path, along the geodesic:
  const CPose3D a(TPose3D(0, 0, 0, 3.0, 0.1, 0.2));
  const CPose3D b(TPose3D(0, 0, 0, -3.0, 0.1, 0.2));
  const Eigen::AngleAxisd d(
      a.getRotationMatrix().asEigen().transpose() * b.getRotationMatrix().asEigen());
  const Eigen::Matrix3d Rmid = a.getRotationMatrix().asEigen() *
                               Eigen::AngleAxisd(0.5 * d.angle(), d.axis()).toRotationMatrix();
  EXPECT_LT((CPose3D(out).getRotationMatrix().asEigen() - Rmid).array().abs().maxCoeff(), 1e-9);
  EXPECT_NEAR(out.x, 0.5, 1e-9);
}

namespace
{
double rotDist(const mrpt::poses::CPose3D& a, const mrpt::poses::CPose3D& b)
{
  return (a.getRotationMatrix().asEigen() - b.getRotationMatrix().asEigen()).cwiseAbs().maxCoeff();
}
}  // namespace

// Interpolated values of all methods for a constant-velocity motion: linear
// translation and a constant yaw rate across +-pi, which all methods must
// reproduce exactly.
TEST(CPose3DInterpolator, AllMethodsValuesConstantVelocity)
{
  using namespace mrpt::poses;
  using mrpt::math::TPose3D;

  const auto t0 = mrpt::Clock::fromDouble(1000.0);
  const double dt = 0.1;
  const auto truth = [](double t)
  { return TPose3D(1.0 + 2.0 * t, -0.5 * t, 0.3 * t, 3.0 + 0.5 * t, 0.0, 0.0); };
  CPose3DInterpolator path;
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
      TPose3D p;
      bool valid = false;
      path.interpolate(t0 + std::chrono::microseconds(static_cast<int64_t>(tq * 1e6)), p, valid);
      ASSERT_TRUE(valid);
      const TPose3D gt = truth(tq);
      EXPECT_NEAR(p.x, gt.x, 1e-6) << "method=" << static_cast<int>(m) << " t=" << tq;
      EXPECT_NEAR(p.y, gt.y, 1e-6);
      EXPECT_NEAR(p.z, gt.z, 1e-6);
      EXPECT_LT(rotDist(CPose3D(p), CPose3D(gt)), 1e-6)
          << "method=" << static_cast<int>(m) << " t=" << tq << " p=" << p;
      EXPECT_GE(p.yaw, -M_PI);
      EXPECT_LT(p.yaw, M_PI);
    }
  }
}

// For a constant angular velocity about a general axis, Slerp reproduces the
// rotation exactly, while the per-angle methods only approximate it.
TEST(CPose3DInterpolator, SlerpExactForConstantAngularVelocity)
{
  using namespace mrpt::poses;
  using mrpt::math::TPose3D;
  using SO3 = mrpt::poses::Lie::SO<3>;

  const auto t0 = mrpt::Clock::fromDouble(1000.0);
  const double dt = 0.1;
  const CPose3D R0(0, 0, 0, 0.4, -0.3, 0.8);
  // The rotation passes over pitch=90 deg (gimbal lock) between samples 2 and 3:
  SO3::tangent_vector w;
  w[0] = 0.0;
  w[1] = 1.0;
  w[2] = 0.2;
  const auto truth = [&](double t)
  {
    SO3::tangent_vector v = w;
    v *= t * 8.0;
    return R0 + CPose3D(SO3::exp(v), mrpt::math::CVectorFixedDouble<3>());
  };
  CPose3DInterpolator path;
  for (int i = 0; i < 6; i++)
  {
    path.insert(mrpt::Clock::fromDouble(1000.0 + i * dt), truth(i * dt).asTPose());
  }
  for (const auto m : {imLinearSlerp, imSplineSlerp})
  {
    path.setInterpolationMethod(m);
    for (const double tq : {0.12, 0.25, 0.29})
    {
      CPose3D p;
      bool valid = false;
      path.interpolate(t0 + std::chrono::microseconds(static_cast<int64_t>(tq * 1e6)), p, valid);
      ASSERT_TRUE(valid);
      EXPECT_LT(rotDist(p, truth(tq)), 1e-6) << "method=" << static_cast<int>(m) << " t=" << tq;
    }
  }
}
