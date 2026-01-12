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
#include <mrpt/img/TCamera.h>
#include <mrpt/img/camera_geometry.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose3D.h>

#include <cmath>

using namespace mrpt;
using namespace mrpt::img;
using namespace mrpt::img::camera_geometry;
using namespace mrpt::math;

// ============================================================================
//  Test Data (Ground truth values)
// ============================================================================

namespace test_data
{
// Standard pinhole camera parameters
struct StandardCamera
{
  static constexpr double fx = 800.0;
  static constexpr double fy = 800.0;
  static constexpr double cx = 320.0;
  static constexpr double cy = 240.0;
  static constexpr uint32_t width = 640;
  static constexpr uint32_t height = 480;
};

// Plumb-bob distortion coefficients (typical moderate distortion)
struct PlumbBobDistortion
{
  static constexpr double k1 = -0.28340811;
  static constexpr double k2 = 0.07395907;
  static constexpr double p1 = 0.00019359;
  static constexpr double p2 = 0.00019359;
  static constexpr double k3 = 0.0;
};

// Fish-eye distortion coefficients
struct FisheyeDistortion
{
  static constexpr double k1 = 0.1;
  static constexpr double k2 = 0.01;
  static constexpr double k3 = 0.001;
  static constexpr double k4 = 0.0001;
};

// Known projection results (generated with OpenCV for validation)
// Format: {3D point (X,Y,Z), expected pixel (u,v)}
struct ProjectionTestCase
{
  TPoint3D point3D;
  TPixelCoordf expectedPixel;
};

// Undistortion test cases
// Format: {distorted pixel, expected undistorted pixel}
struct UndistortionTestCase
{
  TPixelCoordf distortedPixel;
  TPixelCoordf expectedUndistortedPixel;
};

// Ground truth data for plumb_bob model (generated with generate_test_data_opencv.cpp)
const std::vector<ProjectionTestCase> plumbBobProjections = {
    { {1.0000, 0.5000, 2.0000}, {687.6276f, 423.8380f}},
    { {0.0000, 0.0000, 1.0000}, {320.0000f, 240.0000f}},
    {{-0.5000, 0.3000, 1.5000},  {64.3404f, 393.4332f}},
    { {2.0000, 1.0000, 3.0000}, {781.8274f, 470.9567f}},
    {{0.8000, -0.6000, 2.5000},  {564.9090f, 56.3616f}}
};

const std::vector<UndistortionTestCase> plumbBobUndistortion = {
    {{320.0000f, 240.0000f}, {320.0000f, 240.0000f}},
    {{400.0000f, 300.0000f}, {400.3495f, 300.2615f}},
    {{200.0000f, 150.0000f}, {198.7605f, 149.0689f}},
    {{500.0000f, 400.0000f}, {504.8348f, 404.2959f}},
    {{100.0000f, 100.0000f},   {92.8667f, 95.4540f}}
};

const std::vector<ProjectionTestCase> fisheyeProjections = {
    {{0.5000, 0.0000, 1.0000}, {699.0669f, 240.0000f}},
    {{0.0000, 0.5000, 1.0000}, {320.0000f, 619.0669f}},
    {{0.5000, 0.5000, 1.0000}, {681.8763f, 601.8763f}},
    {{0.8000, 0.8000, 1.0000}, {836.1346f, 756.1346f}}
};

// Normalized coordinate distortion test cases
// Format: {normalized (x,y), expected distorted (xd,yd)}
struct NormalizedDistortionTest
{
  double x, y;
  double xd_expected, yd_expected;
};

const std::vector<NormalizedDistortionTest> plumbBobNormalizedTests = {
    { 0.0000, 0.0000,  0.0000, 0.0000},
    { 0.5000, 0.0000,  0.4670, 0.0000},
    { 0.0000, 0.5000,  0.0000, 0.4670},
    { 0.5000, 0.5000,  0.4387, 0.4387},
    {-0.3000, 0.2000, -0.2893, 0.1929},
    { 1.0000, 0.0000,  0.7911, 0.0002}
};
}  // namespace test_data

// ============================================================================
//  Helper Functions
// ============================================================================
namespace
{
TCamera createStandardCamera()
{
  TCamera cam;
  cam.ncols = test_data::StandardCamera::width;
  cam.nrows = test_data::StandardCamera::height;
  cam.setIntrinsicParamsFromValues(
      test_data::StandardCamera::fx, test_data::StandardCamera::fy, test_data::StandardCamera::cx,
      test_data::StandardCamera::cy);
  cam.distortion = DistortionModel::none;
  return cam;
}

TCamera createPlumbBobCamera()
{
  TCamera cam = createStandardCamera();
  cam.setDistortionPlumbBob(
      test_data::PlumbBobDistortion::k1, test_data::PlumbBobDistortion::k2,
      test_data::PlumbBobDistortion::p1, test_data::PlumbBobDistortion::p2,
      test_data::PlumbBobDistortion::k3);
  return cam;
}

TCamera createFisheyeCamera()
{
  TCamera cam = createStandardCamera();
  cam.setDistortionKannalaBrandt(
      test_data::FisheyeDistortion::k1, test_data::FisheyeDistortion::k2,
      test_data::FisheyeDistortion::k3, test_data::FisheyeDistortion::k4);
  return cam;
}

void expectPixelNear(
    const TPixelCoordf& actual, const TPixelCoordf& expected, float tolerance = 0.5f)
{
  EXPECT_NEAR(actual.x, expected.x, tolerance) << "Pixel x-coordinate mismatch";
  EXPECT_NEAR(actual.y, expected.y, tolerance) << "Pixel y-coordinate mismatch";
}

void expectPointNear(const TPoint2D& actual, const TPoint2D& expected, double tolerance = 1e-4)
{
  EXPECT_NEAR(actual.x, expected.x, tolerance) << "Point x-coordinate mismatch";
  EXPECT_NEAR(actual.y, expected.y, tolerance) << "Point y-coordinate mismatch";
}
}  // namespace

// ============================================================================
//  Projection Tests (No Distortion)
// ============================================================================

TEST(CameraGeometry, ProjectPointNoDist_Identity)
{
  TCamera cam = createStandardCamera();
  TPose3D cameraPose = TPose3D::Identity();

  // Point at (1,0,2) should project to center-right
  TPoint3D pt(1.0, 0.0, 2.0);
  TPixelCoordf pixel = projectPoint<false>(cam, cameraPose, pt);

  // Expected: u = 800*(1/2) + 320 = 720, v = 800*(0/2) + 240 = 240
  expectPixelNear(pixel, {720.0f, 240.0f});
}

TEST(CameraGeometry, ProjectPointNoDist_CameraOffset)
{
  TCamera cam = createStandardCamera();
  // Camera at (1, 0, 0) looking along +X
  TPose3D cameraPose(1.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  // Point at (3, 0, 0) is 2m ahead of camera
  TPoint3D pt(3.0, 0.5, 0.0);

  std::vector<TPoint3D> points = {pt};
  std::vector<TPixelCoordf> pixels;

  projectPoints(points, cameraPose, cam.intrinsicParams, pixels);

  ASSERT_EQ(pixels.size(), 1);
  // Point is (2, 0.5, 0) in camera frame
  // Expected: u = 320 + 800*(2/0) -> invalid (Z=0), but let's use offset
}

TEST(CameraGeometry, ProjectPoints_Batch)
{
  TCamera cam = createStandardCamera();
  TPose3D cameraPose = TPose3D::Identity();

  std::vector<TPoint3D> points = {
      { 0.0,  0.0, 1.0}, // center
      { 0.5,  0.0, 1.0}, // right
      { 0.0,  0.5, 1.0}, // down
      {-0.5, -0.5, 1.0}  // up-left
  };

  std::vector<TPixelCoordf> pixels;
  projectPoints(points, cameraPose, cam.intrinsicParams, pixels);

  ASSERT_EQ(pixels.size(), 4);
  expectPixelNear(pixels[0], {320.0f, 240.0f});   // center
  expectPixelNear(pixels[1], {720.0f, 240.0f});   // right
  expectPixelNear(pixels[2], {320.0f, 640.0f});   // down
  expectPixelNear(pixels[3], {-80.0f, -160.0f});  // up-left (negative pixels)
}

TEST(CameraGeometry, ProjectPoint_BehindCamera)
{
  TCamera cam = createStandardCamera();
  TPose3D cameraPose = TPose3D::Identity();

  std::vector<TPoint3D> points = {
      {0.0, 0.0, -1.0}
  };  // Behind camera
  std::vector<TPixelCoordf> pixels;

  projectPoints(points, cameraPose, cam.intrinsicParams, pixels, false);

  ASSERT_EQ(pixels.size(), 1);
  EXPECT_EQ(pixels[0].x, -1.0f) << "Points behind camera should be marked (-1,-1)";
  EXPECT_EQ(pixels[0].y, -1.0f);
}

// ============================================================================
//  Projection Tests (With Distortion)
// ============================================================================

TEST(CameraGeometry, ProjectPointsPlumbBob_GroundTruth)
{
  TCamera cam = createPlumbBobCamera();
  TPose3D cameraPose = TPose3D::Identity();

  for (const auto& testCase : test_data::plumbBobProjections)
  {
    std::vector<TPoint3D> points = {testCase.point3D};
    std::vector<TPixelCoordf> pixels;

    projectPoints_with_distortion(points, cameraPose, cam, pixels);

    ASSERT_EQ(pixels.size(), 1);
    expectPixelNear(pixels[0], testCase.expectedPixel, 1.0f);
  }
}

TEST(CameraGeometry, ProjectPointsFisheye_GroundTruth)
{
  TCamera cam = createFisheyeCamera();
  TPose3D cameraPose = TPose3D::Identity();

  for (const auto& testCase : test_data::fisheyeProjections)
  {
    std::vector<TPoint3D> points = {testCase.point3D};
    std::vector<TPixelCoordf> pixels;

    projectPoints_with_distortion(points, cameraPose, cam, pixels);

    ASSERT_EQ(pixels.size(), 1);
    expectPixelNear(pixels[0], testCase.expectedPixel, 1.0f);
  }
}

TEST(CameraGeometry, ProjectPoint_CenterNoDistortion)
{
  TCamera cam = createPlumbBobCamera();

  // Point along optical axis should have no distortion
  TPoint3D pt(0.0, 0.0, 1.0);
  TPixelCoordf pixel;

  projectPoint_with_distortion(pt, cam, pixel);

  expectPixelNear(pixel, {320.0f, 240.0f}, 0.1f);
}

// ============================================================================
//  Undistortion Tests
// ============================================================================

TEST(CameraGeometry, UndistortPoint_PlumbBob_GroundTruth)
{
  TCamera cam = createPlumbBobCamera();

  for (const auto& testCase : test_data::plumbBobUndistortion)
  {
    TPixelCoordf undistorted;
    undistort_point(testCase.distortedPixel, undistorted, cam);

    expectPixelNear(undistorted, testCase.expectedUndistortedPixel, 0.5f);
  }
}

TEST(CameraGeometry, UndistortPoints_Batch)
{
  TCamera cam = createPlumbBobCamera();

  std::vector<TPixelCoordf> distorted;
  std::vector<TPixelCoordf> expectedUndistorted;

  for (const auto& testCase : test_data::plumbBobUndistortion)
  {
    distorted.push_back(testCase.distortedPixel);
    expectedUndistorted.push_back(testCase.expectedUndistortedPixel);
  }

  std::vector<TPixelCoordf> undistorted;
  undistort_points(distorted, undistorted, cam);

  ASSERT_EQ(undistorted.size(), expectedUndistorted.size());
  for (size_t i = 0; i < undistorted.size(); ++i)
  {
    expectPixelNear(undistorted[i], expectedUndistorted[i], 0.5f);
  }
}

TEST(CameraGeometry, UndistortPoint_NoDist_Identity)
{
  TCamera cam = createStandardCamera();  // No distortion

  TPixelCoordf distorted(400.0f, 300.0f);
  TPixelCoordf undistorted;

  undistort_point(distorted, undistorted, cam);

  expectPixelNear(undistorted, distorted, 0.01f);
}

// ============================================================================
//  Undistort to Unit Plane Tests
// ============================================================================

TEST(CameraGeometry, UndistortToUnitPlane_Center)
{
  TCamera cam = createPlumbBobCamera();

  std::vector<TPixelCoordf> pixels = {
      {320.0f, 240.0f}
  };  // Principal point
  std::vector<TPoint2D> normalized;

  undistort_points_to_unit_plane(pixels, normalized, cam);

  ASSERT_EQ(normalized.size(), 1);
  // Principal point maps to (0, 0) in normalized coordinates
  expectPointNear(normalized[0], {0.0, 0.0}, 1e-4);
}

TEST(CameraGeometry, UndistortToUnitPlane_Consistency)
{
  TCamera cam = createPlumbBobCamera();

  // Test round-trip: pixel -> normalized -> pixel
  std::vector<TPixelCoordf> originalPixels = {
      {400.0f, 300.0f},
      {200.0f, 150.0f},
      {500.0f, 400.0f}
  };

  std::vector<TPoint2D> normalized;
  undistort_points_to_unit_plane(originalPixels, normalized, cam);

  // Project normalized coords back to pixels (without distortion)
  TCamera camNoDist = createStandardCamera();
  for (size_t i = 0; i < normalized.size(); ++i)
  {
    TPoint3D pt3D(normalized[i].x, normalized[i].y, 1.0);
    TPixelCoordf pixel = projectPoint(pt3D, camNoDist);

    // Should be close to undistorted version
    TPixelCoordf undistorted;
    undistort_point(originalPixels[i], undistorted, cam);

    expectPixelNear(pixel, undistorted, 0.5f);
  }
}

// ============================================================================
//  Low-Level Distortion Function Tests
// ============================================================================

TEST(CameraGeometry, DistortionPlumbBob_Normalized_GroundTruth)
{
  std::array<double, 8> dist = {};
  dist[0] = test_data::PlumbBobDistortion::k1;
  dist[1] = test_data::PlumbBobDistortion::k2;
  dist[2] = test_data::PlumbBobDistortion::p1;
  dist[3] = test_data::PlumbBobDistortion::p2;
  dist[4] = test_data::PlumbBobDistortion::k3;
  dist[5] = 0.0;
  dist[6] = 0.0;
  dist[7] = 0.0;

  for (const auto& testCase : test_data::plumbBobNormalizedTests)
  {
    double xd = 0;
    double yd = 0;
    distortion::apply_plumb_bob(testCase.x, testCase.y, dist, xd, yd);

    EXPECT_NEAR(xd, testCase.xd_expected, 1e-3)
        << "For input (" << testCase.x << ", " << testCase.y << ")";
    EXPECT_NEAR(yd, testCase.yd_expected, 1e-3)
        << "For input (" << testCase.x << ", " << testCase.y << ")";
  }
}

TEST(CameraGeometry, DistortionPlumbBob_RoundTrip)
{
  std::array<double, 8> dist = {};
  dist[0] = test_data::PlumbBobDistortion::k1;
  dist[1] = test_data::PlumbBobDistortion::k2;
  dist[2] = test_data::PlumbBobDistortion::p1;
  dist[3] = test_data::PlumbBobDistortion::p2;
  dist[4] = test_data::PlumbBobDistortion::k3;
  dist[5] = 0.0;
  dist[6] = 0.0;
  dist[7] = 0.0;

  std::vector<std::pair<double, double>> testPoints = {
      { 0.0,  0.0},
      { 0.5,  0.0},
      { 0.0,  0.5},
      { 0.5,  0.5},
      {-0.3,  0.2},
      { 0.8, -0.4}
  };

  for (const auto& [x, y] : testPoints)
  {
    // Apply distortion
    double xd = 0;
    double yd = 0;
    distortion::apply_plumb_bob(x, y, dist, xd, yd);

    // Remove distortion
    double x_recovered = 0;
    double y_recovered = 0;
    distortion::remove_plumb_bob(xd, yd, dist, x_recovered, y_recovered);

    EXPECT_NEAR(x_recovered, x, 1e-3) << "Round-trip failed for x=" << x;
    EXPECT_NEAR(y_recovered, y, 1e-3) << "Round-trip failed for y=" << y;
  }
}

TEST(CameraGeometry, DistortionKannalaBrandt_RoundTrip)
{
  std::array<double, 8> dist = {};
  dist[0] = 0.1;
  dist[1] = 0.01;
  dist[2] = 0.0;
  dist[3] = 0.0;
  dist[4] = 0.001;
  dist[5] = 0.0001;
  dist[6] = 0.0;
  dist[7] = 0.0;

  std::vector<std::pair<double, double>> testPoints = {
      {0.0, 0.0},
      {0.3, 0.0},
      {0.0, 0.3},
      {0.5, 0.5},
      {0.8, 0.8},
      {1.0, 0.0}
  };

  for (const auto& [x, y] : testPoints)
  {
    // Apply distortion
    double xd = 0;
    double yd = 0;
    distortion::apply_kannala_brandt(x, y, dist, xd, yd);

    // Remove distortion
    double x_recovered = 0;
    double y_recovered = 0;
    distortion::remove_kannala_brandt(xd, yd, dist, x_recovered, y_recovered);

    EXPECT_NEAR(x_recovered, x, 1e-3) << "Round-trip failed for x=" << x;
    EXPECT_NEAR(y_recovered, y, 1e-3) << "Round-trip failed for y=" << y;
  }
}

TEST(CameraGeometry, DistortionKannalaBrandt_Origin)
{
  std::array<double, 8> dist = {};
  dist.fill(0.1);  // Any values

  double xd = 0;
  double yd = 0;
  distortion::apply_kannala_brandt(0.0, 0.0, dist, xd, yd);

  EXPECT_NEAR(xd, 0.0, 1e-10);
  EXPECT_NEAR(yd, 0.0, 1e-10);
}

// ============================================================================
//  Edge Cases and Error Handling
// ============================================================================

TEST(CameraGeometry, ProjectPoint_ZeroDepth)
{
  TCamera cam = createStandardCamera();
  TPoint3D pt(1.0, 1.0, 0.0);  // Z = 0

  // This should trigger assertion or return invalid
  std::vector<TPoint3D> points = {pt};
  std::vector<TPixelCoordf> pixels;

  TPose3D pose = TPose3D::Identity();

  // With acceptPointsBehind=false, should mark as invalid
  projectPoints(points, pose, cam.intrinsicParams, pixels, false);

  ASSERT_EQ(pixels.size(), 1);
  EXPECT_EQ(pixels[0].x, -1.0f);
  EXPECT_EQ(pixels[0].y, -1.0f);
}

TEST(CameraGeometry, EmptyInputs)
{
  TCamera cam = createStandardCamera();
  TPose3D pose = TPose3D::Identity();

  std::vector<TPoint3D> emptyPoints;
  std::vector<TPixelCoordf> pixels;

  projectPoints(emptyPoints, pose, cam.intrinsicParams, pixels);
  EXPECT_EQ(pixels.size(), 0);

  projectPoints_with_distortion(emptyPoints, pose, cam, pixels);
  EXPECT_EQ(pixels.size(), 0);
}

// ============================================================================
//  Performance/Stress Tests
// ============================================================================

TEST(CameraGeometry, ProjectLargePointCloud)
{
  TCamera cam = createPlumbBobCamera();
  TPose3D pose = TPose3D::Identity();

  // Create 1000 random points
  std::vector<TPoint3D> points;
  points.reserve(1000);
  for (int i = 0; i < 1000; ++i)
  {
    points.emplace_back(
        (i % 10) * 0.1 - 0.5, (i / 10 % 10) * 0.1 - 0.5,
        1.0 + (static_cast<double>(i) / 100) * 0.1);
  }

  std::vector<TPixelCoordf> pixels;
  projectPoints_with_distortion(points, pose, cam, pixels);

  EXPECT_EQ(pixels.size(), 1000);

  // All points should be valid (not behind camera)
  for (const auto& pixel : pixels)
  {
    EXPECT_NE(pixel.x, -1.0f);
    EXPECT_NE(pixel.y, -1.0f);
  }
}