/*
 * Helper program to generate ground truth test data using OpenCV
 *
 * This generates reference values for camera_geometry_unittest.cpp
 * Run this once to generate the test data, then embed results into the unit tests.
 *
 * Compile with:
 * g++ -o generate_test_data generate_test_data_opencv.cpp `pkg-config --cflags --libs opencv4`
 */

#include <iomanip>
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <vector>

// Camera parameters (must match test_data::StandardCamera in unit tests)
const double fx = 800.0;
const double fy = 800.0;
const double cx = 320.0;
const double cy = 240.0;

// Distortion parameters (must match test_data::PlumbBobDistortion)
const double k1 = -0.28340811;
const double k2 = 0.07395907;
const double p1 = 0.00019359;
const double p2 = 0.00019359;
const double k3 = 0.0;

void generateProjectionTestData()
{
  std::cout << "\n=== PROJECTION TEST DATA (Plumb-Bob) ===\n";
  std::cout << std::setprecision(4) << std::fixed;

  cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);

  cv::Mat distCoeffs = (cv::Mat_<double>(5, 1) << k1, k2, p1, p2, k3);

  // Test 3D points (in camera frame)
  std::vector<cv::Point3d> points3D = {
      { 1.0,  0.5, 2.0},
      { 0.0,  0.0, 1.0},
      {-0.5,  0.3, 1.5},
      { 2.0,  1.0, 3.0},
      { 0.8, -0.6, 2.5}
  };

  // Identity rotation/translation (points already in camera frame)
  cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
  cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);

  std::vector<cv::Point2d> projectedPixels;
  cv::projectPoints(points3D, rvec, tvec, cameraMatrix, distCoeffs, projectedPixels);

  std::cout << "const std::vector<ProjectionTestCase> plumbBobProjections = {\n";
  for (size_t i = 0; i < points3D.size(); ++i)
  {
    std::cout << "    {{" << points3D[i].x << ", " << points3D[i].y << ", " << points3D[i].z
              << "}, {" << projectedPixels[i].x << "f, " << projectedPixels[i].y << "f}}";
    if (i < points3D.size() - 1) std::cout << ",";
    std::cout << "\n";
  }
  std::cout << "};\n";
}

void generateUndistortionTestData()
{
  std::cout << "\n=== UNDISTORTION TEST DATA (Plumb-Bob) ===\n";
  std::cout << std::setprecision(4) << std::fixed;

  cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);

  cv::Mat distCoeffs = (cv::Mat_<double>(5, 1) << k1, k2, p1, p2, k3);

  // Test distorted pixels
  std::vector<cv::Point2f> distortedPixels = {
      {320.0f, 240.0f}, // center
      {400.0f, 300.0f},
      {200.0f, 150.0f},
      {500.0f, 400.0f},
      {100.0f, 100.0f}
  };

  std::vector<cv::Point2f> undistortedPixels;
  cv::undistortPoints(
      distortedPixels, undistortedPixels, cameraMatrix, distCoeffs, cv::noArray(), cameraMatrix);

  std::cout << "const std::vector<UndistortionTestCase> plumbBobUndistortions = {\n";
  for (size_t i = 0; i < distortedPixels.size(); ++i)
  {
    std::cout << "    {{" << distortedPixels[i].x << "f, " << distortedPixels[i].y << "f}, {"
              << undistortedPixels[i].x << "f, " << undistortedPixels[i].y << "f}}";
    if (i < distortedPixels.size() - 1) std::cout << ",";
    std::cout << "\n";
  }
  std::cout << "};\n";
}

void generateNormalizedDistortionData()
{
  std::cout << "\n=== NORMALIZED DISTORTION TEST DATA ===\n";
  std::cout << std::setprecision(4) << std::fixed;

  // Normalized coordinates (x, y) at z=1
  std::vector<cv::Point2d> normalized = {
      { 0.0, 0.0},
      { 0.5, 0.0},
      { 0.0, 0.5},
      { 0.5, 0.5},
      {-0.3, 0.2},
      { 1.0, 0.0}
  };

  std::cout << "const std::vector<NormalizedDistortionTest> plumbBobNormalizedTests = {\n";

  for (const auto& pt : normalized)
  {
    double x = pt.x;
    double y = pt.y;

    // Apply plumb_bob distortion manually
    double r2 = x * x + y * y;
    double r4 = r2 * r2;
    double r6 = r2 * r4;

    double radial = (1.0 + k1 * r2 + k2 * r4 + k3 * r6);
    double dx_tangential = 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
    double dy_tangential = p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;

    double xd = x * radial + dx_tangential;
    double yd = y * radial + dy_tangential;

    std::cout << "    {" << x << ", " << y << ", " << xd << ", " << yd << "}";
    if (pt.x != 1.0 || pt.y != 0.0) std::cout << ",";
    std::cout << "\n";
  }
  std::cout << "};\n";
}

void generateFisheyeTestData()
{
  std::cout << "\n=== FISHEYE (Kannala-Brandt) TEST DATA ===\n";
  std::cout << std::setprecision(4) << std::fixed;

  // Note: For fisheye, you would use cv::fisheye::projectPoints
  // and cv::fisheye::undistortPoints

  // Sample fisheye distortion coefficients
  const double fk1 = 0.1;
  const double fk2 = 0.01;
  const double fk3 = 0.001;
  const double fk4 = 0.0001;

  cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);

  cv::Mat fisheyeDistCoeffs = (cv::Mat_<double>(4, 1) << fk1, fk2, fk3, fk4);

  std::vector<cv::Point3d> points3D = {
      {0.5, 0.0, 1.0},
      {0.0, 0.5, 1.0},
      {0.5, 0.5, 1.0},
      {0.8, 0.8, 1.0}
  };

  cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
  cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);

  std::vector<cv::Point2d> projectedPixels;
  cv::fisheye::projectPoints(
      points3D, projectedPixels, rvec, tvec, cameraMatrix, fisheyeDistCoeffs);

  std::cout << "Fisheye distortion coefficients: k1=" << fk1 << ", k2=" << fk2 << ", k3=" << fk3
            << ", k4=" << fk4 << "\n";
  std::cout << "const std::vector<ProjectionTestCase> fisheyeProjections = {\n";
  for (size_t i = 0; i < points3D.size(); ++i)
  {
    std::cout << "    {{" << points3D[i].x << ", " << points3D[i].y << ", " << points3D[i].z
              << "}, {" << projectedPixels[i].x << "f, " << projectedPixels[i].y << "f}}";
    if (i < points3D.size() - 1) std::cout << ",";
    std::cout << "\n";
  }
  std::cout << "};\n";
}

void validateRoundTrip()
{
  std::cout << "\n=== ROUND-TRIP VALIDATION ===\n";

  cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);

  cv::Mat distCoeffs = (cv::Mat_<double>(5, 1) << k1, k2, p1, p2, k3);

  // Test point
  cv::Point3d point3D(1.0, 0.5, 2.0);

  // Project
  cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
  cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);

  std::vector<cv::Point3d> points = {point3D};
  std::vector<cv::Point2d> pixels;
  cv::projectPoints(points, rvec, tvec, cameraMatrix, distCoeffs, pixels);

  std::cout << "Original 3D point: (" << point3D.x << ", " << point3D.y << ", " << point3D.z
            << ")\n";
  std::cout << "Projected pixel: (" << pixels[0].x << ", " << pixels[0].y << ")\n";

  // Undistort
  std::vector<cv::Point2f> distorted = {
      {static_cast<float>(pixels[0].x), static_cast<float>(pixels[0].y)}
  };
  std::vector<cv::Point2f> undistorted;
  cv::undistortPoints(
      distorted, undistorted, cameraMatrix, distCoeffs, cv::noArray(), cameraMatrix);

  std::cout << "Undistorted pixel: (" << undistorted[0].x << ", " << undistorted[0].y << ")\n";

  // Expected undistorted (no distortion projection)
  cv::Mat noDistCoeffs = cv::Mat::zeros(5, 1, CV_64F);
  std::vector<cv::Point2d> pixelsNoDist;
  cv::projectPoints(points, rvec, tvec, cameraMatrix, noDistCoeffs, pixelsNoDist);

  std::cout << "Expected (no distortion): (" << pixelsNoDist[0].x << ", " << pixelsNoDist[0].y
            << ")\n";
  std::cout << "Difference: (" << (undistorted[0].x - pixelsNoDist[0].x) << ", "
            << (undistorted[0].y - pixelsNoDist[0].y) << ")\n";
}

void printUsageInstructions()
{
  std::cout << R"(
============================================================
USAGE INSTRUCTIONS
============================================================

1. Copy the generated test data into camera_geometry_unittest.cpp
   in the appropriate namespace test_data sections.

2. The test data includes:
   - Projection test cases (3D points -> pixels)
   - Undistortion test cases (distorted -> undistorted pixels)
   - Normalized coordinate distortion tests
   - Fisheye projection test cases

3. Verify the camera parameters match between this generator
   and the unit test file.

4. The round-trip validation helps ensure OpenCV implementation
   is consistent with expected behavior.

============================================================
)";
}

int main()
{
  std::cout << "Generating camera geometry test data using OpenCV...\n";
  std::cout << "OpenCV version: " << CV_VERSION << "\n";

  printUsageInstructions();

  generateProjectionTestData();
  generateUndistortionTestData();
  generateNormalizedDistortionData();
  generateFisheyeTestData();
  validateRoundTrip();

  std::cout << "\n=== GENERATION COMPLETE ===\n";
  std::cout << "Copy the data blocks above into camera_geometry_unittest.cpp\n";

  return 0;
}