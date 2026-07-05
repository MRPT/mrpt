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
#include <mrpt/math/ransac_applications.h>
#include <mrpt/random/RandomGenerators.h>

using namespace mrpt::math;

TEST(RansacApplications, Detect2DLinesEmptyInput)
{
  CVectorDouble x;
  CVectorDouble y;
  std::vector<std::pair<size_t, TLine2D>> lines;
  mrpt::math::ransac_detect_2D_lines(x, y, lines, 0.1, 10);
  EXPECT_TRUE(lines.empty());
}

TEST(RansacApplications, Detect2DLinesFindsOneLine)
{
  mrpt::random::getRandomGenerator().randomize(1234);

  constexpr size_t N = 60;
  CVectorDouble x(N);
  CVectorDouble y(N);
  for (size_t i = 0; i < N; i++)
  {
    const double xi = static_cast<double>(i) * 0.2;
    const int ii = static_cast<int>(i);
    x[ii] = xi;
    y[ii] = 2.0 * xi + 1.0 + mrpt::random::getRandomGenerator().drawGaussian1D(0, 0.005);
  }

  std::vector<std::pair<size_t, TLine2D>> lines;
  mrpt::math::ransac_detect_2D_lines(
      x, y, lines, /*threshold=*/0.1, /*min_inliers_for_valid_line=*/20);

  ASSERT_GE(lines.size(), 1U);
  EXPECT_GE(lines[0].first, 20U);
  // Points on the true line y=2x+1 should be close to zero distance
  EXPECT_LT(lines[0].second.distance(TPoint2D(0.0, 1.0)), 0.2);
}

TEST(RansacApplications, Detect2DLinesFindsTwoLines)
{
  mrpt::random::getRandomGenerator().randomize(4321);

  constexpr size_t N_PER_LINE = 40;
  CVectorDouble x(2 * N_PER_LINE);
  CVectorDouble y(2 * N_PER_LINE);
  for (size_t i = 0; i < N_PER_LINE; i++)
  {
    const double xi = static_cast<double>(i) * 0.1;
    const int ii = static_cast<int>(i);
    // Line 1: y = 0
    x[ii] = xi;
    y[ii] = 0.0 + mrpt::random::getRandomGenerator().drawGaussian1D(0, 0.005);
    // Line 2: y = 5
    x[static_cast<int>(N_PER_LINE + i)] = xi;
    y[static_cast<int>(N_PER_LINE + i)] =
        5.0 + mrpt::random::getRandomGenerator().drawGaussian1D(0, 0.005);
  }

  std::vector<std::pair<size_t, TLine2D>> lines;
  mrpt::math::ransac_detect_2D_lines(
      x, y, lines, /*threshold=*/0.1, /*min_inliers_for_valid_line=*/15);

  EXPECT_GE(lines.size(), 2U);
}

TEST(RansacApplications, Detect3DPlanesEmptyInput)
{
  CVectorDouble x;
  CVectorDouble y;
  CVectorDouble z;
  std::vector<std::pair<size_t, TPlane>> planes;
  mrpt::math::ransac_detect_3D_planes(x, y, z, planes, 0.1, 10);
  EXPECT_TRUE(planes.empty());
}

TEST(RansacApplications, Detect3DPlanesFindsOnePlane)
{
  mrpt::random::getRandomGenerator().randomize(9999);

  constexpr size_t N = 100;
  CVectorDouble x(N);
  CVectorDouble y(N);
  CVectorDouble z(N);
  for (size_t i = 0; i < N; i++)
  {
    const int ii = static_cast<int>(i);
    x[ii] = mrpt::random::getRandomGenerator().drawUniform(-5.0, 5.0);
    y[ii] = mrpt::random::getRandomGenerator().drawUniform(-5.0, 5.0);
    // Plane Z=0 with small noise
    z[ii] = mrpt::random::getRandomGenerator().drawGaussian1D(0, 0.005);
  }

  std::vector<std::pair<size_t, TPlane>> planes;
  mrpt::math::ransac_detect_3D_planes(
      x, y, z, planes, /*threshold=*/0.05, /*min_inliers_for_valid_plane=*/30);

  ASSERT_GE(planes.size(), 1U);
  EXPECT_GE(planes[0].first, 30U);
  EXPECT_LT(planes[0].second.distance(TPoint3D(0, 0, 0)), 0.1);
}

TEST(RansacApplications, Detect3DPlanesFloatInstantiation)
{
  mrpt::random::getRandomGenerator().randomize(555);

  constexpr size_t N = 60;
  CVectorFloat x(N);
  CVectorFloat y(N);
  CVectorFloat z(N);
  for (size_t i = 0; i < N; i++)
  {
    const int ii = static_cast<int>(i);
    x[ii] = static_cast<float>(mrpt::random::getRandomGenerator().drawUniform(-5.0, 5.0));
    y[ii] = static_cast<float>(mrpt::random::getRandomGenerator().drawUniform(-5.0, 5.0));
    z[ii] = static_cast<float>(mrpt::random::getRandomGenerator().drawGaussian1D(0, 0.005));
  }

  std::vector<std::pair<size_t, TPlane>> planes;
  mrpt::math::ransac_detect_3D_planes(
      x, y, z, planes, /*threshold=*/0.05, /*min_inliers_for_valid_plane=*/20);

  EXPECT_GE(planes.size(), 1U);
}

TEST(RansacApplications, Detect2DLinesFloatInstantiation)
{
  mrpt::random::getRandomGenerator().randomize(777);

  constexpr size_t N = 40;
  CVectorFloat x(N);
  CVectorFloat y(N);
  for (size_t i = 0; i < N; i++)
  {
    const float xi = static_cast<float>(i) * 0.2f;
    const int ii = static_cast<int>(i);
    x[ii] = xi;
    y[ii] = xi + static_cast<float>(mrpt::random::getRandomGenerator().drawGaussian1D(0, 0.005));
  }

  std::vector<std::pair<size_t, TLine2D>> lines;
  mrpt::math::ransac_detect_2D_lines(
      x, y, lines, /*threshold=*/0.1, /*min_inliers_for_valid_line=*/15);

  EXPECT_GE(lines.size(), 1U);
}
