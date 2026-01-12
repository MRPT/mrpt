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

#include <mrpt/img/camera_geometry.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/TPose3DQuat.h>

#include <cmath>

using namespace mrpt;
using namespace mrpt::img;
using namespace mrpt::img::camera_geometry;
using namespace mrpt::math;

// ============================================================================
//  DISTORTION MODEL IMPLEMENTATIONS
// ============================================================================

namespace mrpt::img::camera_geometry::distortion
{

void apply_plumb_bob(double x, double y, const std::array<double, 8>& dist, double& xd, double& yd)
{
  // Radial distortion
  const double r2 = x * x + y * y;
  const double r4 = r2 * r2;
  const double r6 = r2 * r4;

  // Radial distortion factor: (1 + k1*r^2 + k2*r^4 + k3*r^6) / (1 + k4*r^2 + k5*r^4 + k6*r^6)
  const double numerator = 1.0 + dist[0] * r2 + dist[1] * r4 + dist[4] * r6;
  const double denominator = 1.0 + dist[5] * r2 + dist[6] * r4 + dist[7] * r6;
  const double radialFactor = numerator / denominator;

  // Tangential distortion
  const double xy2 = 2.0 * x * y;
  const double dx_tangential = dist[2] * xy2 + dist[3] * (r2 + 2.0 * x * x);
  const double dy_tangential = dist[2] * (r2 + 2.0 * y * y) + dist[3] * xy2;

  // Combined distortion
  xd = x * radialFactor + dx_tangential;
  yd = y * radialFactor + dy_tangential;
}

void apply_kannala_brandt(
    double x, double y, const std::array<double, 8>& dist, double& xd, double& yd)
{
  const double r = std::sqrt(x * x + y * y);

  if (r < 1e-8)
  {
    // Near origin, no distortion
    xd = x;
    yd = y;
    return;
  }

  const double theta = std::atan(r);
  const double theta2 = theta * theta;
  const double theta4 = theta2 * theta2;
  const double theta6 = theta2 * theta4;
  const double theta8 = theta4 * theta4;

  // theta_d = theta * (1 + k1*theta^2 + k2*theta^4 + k3*theta^6 + k4*theta^8)
  const double theta_d =
      theta * (1.0 + dist[0] * theta2 + dist[1] * theta4 + dist[4] * theta6 + dist[5] * theta8);

  const double scale = theta_d / r;
  xd = x * scale;
  yd = y * scale;
}

void remove_plumb_bob(
    double xd, double yd, const std::array<double, 8>& dist, double& x, double& y, int iterations)
{
  // Initial guess: undistorted = distorted
  x = xd;
  y = yd;

  // Iterative refinement
  for (int iter = 0; iter < iterations; ++iter)
  {
    const double r2 = x * x + y * y;

    // Radial distortion factor
    const double numerator = 1.0 + dist[0] * r2 + dist[1] * r2 * r2 + dist[4] * r2 * r2 * r2;
    const double denominator = 1.0 + dist[5] * r2 + dist[6] * r2 * r2 + dist[7] * r2 * r2 * r2;
    const double ic_dist = denominator / numerator;

    // Tangential distortion offsets
    const double deltaX = 2.0 * dist[2] * x * y + dist[3] * (r2 + 2.0 * x * x);
    const double deltaY = dist[2] * (r2 + 2.0 * y * y) + 2.0 * dist[3] * x * y;

    // Refine estimate
    x = (xd - deltaX) * ic_dist;
    y = (yd - deltaY) * ic_dist;
  }
}

void remove_kannala_brandt(
    double xd, double yd, const std::array<double, 8>& dist, double& x, double& y, int iterations)
{
  const double rd = std::sqrt(xd * xd + yd * yd);

  if (rd < 1e-8)
  {
    // Near origin, no distortion
    x = xd;
    y = yd;
    return;
  }

  // Initial guess for theta
  double theta = rd;

  // Newton-Raphson to solve: theta_d = theta*(1 + k1*theta^2 + k2*theta^4 + k3*theta^6 +
  // k4*theta^8)
  for (int iter = 0; iter < iterations; ++iter)
  {
    const double theta2 = theta * theta;
    const double theta4 = theta2 * theta2;
    const double theta6 = theta2 * theta4;
    const double theta8 = theta4 * theta4;

    // f(theta) = theta*(1 + k1*theta^2 + k2*theta^4 + k3*theta^6 + k4*theta^8) - theta_d
    const double poly =
        1.0 + dist[0] * theta2 + dist[1] * theta4 + dist[4] * theta6 + dist[5] * theta8;
    const double f = theta * poly - rd;

    // f'(theta) = poly + theta * d(poly)/d(theta)
    const double dpoly = 2.0 * dist[0] * theta + 4.0 * dist[1] * theta * theta2 +
                         6.0 * dist[4] * theta * theta4 + 8.0 * dist[5] * theta * theta6;
    const double df = poly + theta * dpoly;

    if (std::abs(df) < 1e-10)
    {
      break;
    }

    theta = theta - f / df;
  }

  const double r = std::tan(theta);
  const double scale = r / rd;

  x = xd * scale;
  y = yd * scale;
}

}  // namespace mrpt::img::camera_geometry::distortion

// ============================================================================
//  PROJECTION FUNCTIONS
// ============================================================================

void mrpt::img::camera_geometry::projectPoints(
    const std::vector<mrpt::math::TPoint3D>& points3D,
    const mrpt::math::TPose3D& cameraPose,
    const mrpt::math::CMatrixDouble33& intrinsicParams,
    std::vector<mrpt::img::TPixelCoordf>& projectedPoints,
    bool acceptPointsBehind)
{
  MRPT_START

  ASSERT_(intrinsicParams.rows() == 3 && intrinsicParams.cols() == 3);

  const size_t N = points3D.size();
  projectedPoints.resize(N);

  if (N == 0)
  {
    return;
  }

  const double fx = intrinsicParams(0, 0);
  const double fy = intrinsicParams(1, 1);
  const double cx = intrinsicParams(0, 2);
  const double cy = intrinsicParams(1, 2);

  for (size_t i = 0; i < N; ++i)
  {
    // Transform point to camera frame
    const TPoint3D pt = cameraPose.inverseComposePoint(points3D[i]);

    if (acceptPointsBehind || pt.z > 0)
    {
      // Pinhole projection
      const double inv_z = 1.0 / pt.z;
      projectedPoints[i].x = static_cast<float>(cx + fx * pt.x * inv_z);
      projectedPoints[i].y = static_cast<float>(cy + fy * pt.y * inv_z);
    }
    else
    {
      // Point behind camera
      projectedPoints[i].x = -1.0f;
      projectedPoints[i].y = -1.0f;
    }
  }

  MRPT_END
}

void mrpt::img::camera_geometry::projectPoints_with_distortion(
    const std::vector<mrpt::math::TPoint3D>& points3D,
    const mrpt::math::TPose3D& cameraPose,
    const mrpt::img::TCamera& cameraParams,
    std::vector<mrpt::img::TPixelCoordf>& projectedPoints,
    bool acceptPointsBehind)
{
  MRPT_START

  const size_t N = points3D.size();
  projectedPoints.resize(N);

  if (N == 0)
  {
    return;
  }

  const double fx = cameraParams.fx();
  const double fy = cameraParams.fy();
  const double cx = cameraParams.cx();
  const double cy = cameraParams.cy();

  for (size_t i = 0; i < N; ++i)
  {
    // Transform point to camera frame
    const TPoint3D pt = cameraPose.inverseComposePoint(points3D[i]);

    if (!acceptPointsBehind && pt.z <= 0)
    {
      projectedPoints[i].x = -1.0f;
      projectedPoints[i].y = -1.0f;
      continue;
    }

    // Normalize coordinates
    const double inv_z = 1.0 / pt.z;
    const double x = pt.x * inv_z;
    const double y = pt.y * inv_z;

    double xd = 0;
    double yd = 0;

    // Apply distortion
    switch (cameraParams.distortion)
    {
      case DistortionModel::none:
        xd = x;
        yd = y;
        break;

      case DistortionModel::plumb_bob:
        distortion::apply_plumb_bob(x, y, cameraParams.dist, xd, yd);
        break;

      case DistortionModel::kannala_brandt:
        distortion::apply_kannala_brandt(x, y, cameraParams.dist, xd, yd);
        break;

      default:
        THROW_EXCEPTION_FMT(
            "Unknown distortion model: %d", static_cast<int>(cameraParams.distortion));
    }

    // Project to pixel coordinates
    projectedPoints[i].x = static_cast<float>(cx + fx * xd);
    projectedPoints[i].y = static_cast<float>(cy + fy * yd);
  }

  MRPT_END
}

void mrpt::img::camera_geometry::projectPoints_with_distortion(
    const std::vector<mrpt::math::TPoint3D>& points3D,
    const mrpt::img::TCamera& cameraParams,
    const mrpt::math::TPose3D& cameraPose,
    std::vector<mrpt::img::TPixelCoordf>& projectedPoints,
    bool acceptPointsBehind)
{
  MRPT_START

  const size_t N = points3D.size();
  projectedPoints.resize(N);

  if (N == 0)
  {
    return;
  }

  const double fx = cameraParams.fx();
  const double fy = cameraParams.fy();
  const double cx = cameraParams.cx();
  const double cy = cameraParams.cy();

  for (size_t i = 0; i < N; ++i)
  {
    // Transform point to camera frame
    const TPoint3D pt = cameraPose.inverseComposePoint(points3D[i]);

    if (!acceptPointsBehind && pt.z <= 0)
    {
      projectedPoints[i].x = -1.0f;
      projectedPoints[i].y = -1.0f;
      continue;
    }

    // Normalize coordinates
    const double inv_z = 1.0 / pt.z;
    const double x = pt.x * inv_z;
    const double y = pt.y * inv_z;

    double xd = 0;
    double yd = 0;

    // Apply distortion
    switch (cameraParams.distortion)
    {
      case DistortionModel::none:
        xd = x;
        yd = y;
        break;

      case DistortionModel::plumb_bob:
        distortion::apply_plumb_bob(x, y, cameraParams.dist, xd, yd);
        break;

      case DistortionModel::kannala_brandt:
        distortion::apply_kannala_brandt(x, y, cameraParams.dist, xd, yd);
        break;

      default:
        THROW_EXCEPTION_FMT(
            "Unknown distortion model: %d", static_cast<int>(cameraParams.distortion));
    }

    // Project to pixel coordinates
    projectedPoints[i].x = static_cast<float>(cx + fx * xd);
    projectedPoints[i].y = static_cast<float>(cy + fy * yd);
  }

  MRPT_END
}

void mrpt::img::camera_geometry::projectPoint_with_distortion(
    const mrpt::math::TPoint3D& pointInCamFrame,
    const mrpt::img::TCamera& cameraParams,
    mrpt::img::TPixelCoordf& pixel,
    bool acceptPointsBehind)
{
  MRPT_START

  if (!acceptPointsBehind && pointInCamFrame.z <= 0)
  {
    pixel.x = -1.0f;
    pixel.y = -1.0f;
    return;
  }

  // Normalize coordinates
  const double inv_z = 1.0 / pointInCamFrame.z;
  const double x = pointInCamFrame.x * inv_z;
  const double y = pointInCamFrame.y * inv_z;

  double xd = 0;
  double yd = 0;

  // Apply distortion
  switch (cameraParams.distortion)
  {
    case DistortionModel::none:
      xd = x;
      yd = y;
      break;

    case DistortionModel::plumb_bob:
      distortion::apply_plumb_bob(x, y, cameraParams.dist, xd, yd);
      break;

    case DistortionModel::kannala_brandt:
      distortion::apply_kannala_brandt(x, y, cameraParams.dist, xd, yd);
      break;

    default:
      THROW_EXCEPTION_FMT(
          "Unknown distortion model: %d", static_cast<int>(cameraParams.distortion));
  }

  // Project to pixel coordinates
  const double fx = cameraParams.fx();
  const double fy = cameraParams.fy();
  const double cx = cameraParams.cx();
  const double cy = cameraParams.cy();

  pixel.x = static_cast<float>(cx + fx * xd);
  pixel.y = static_cast<float>(cy + fy * yd);

  MRPT_END
}

// ============================================================================
//  UNDISTORTION FUNCTIONS
// ============================================================================

void mrpt::img::camera_geometry::undistort_points(
    const std::vector<mrpt::img::TPixelCoordf>& distortedPixels,
    std::vector<mrpt::img::TPixelCoordf>& undistortedPixels,
    const mrpt::img::TCamera& cameraParams)
{
  MRPT_START

  const size_t N = distortedPixels.size();
  undistortedPixels.resize(N);

  if (N == 0)
  {
    return;
  }

  const double fx = cameraParams.fx();
  const double fy = cameraParams.fy();
  const double cx = cameraParams.cx();
  const double cy = cameraParams.cy();
  const double inv_fx = 1.0 / fx;
  const double inv_fy = 1.0 / fy;

  for (size_t i = 0; i < N; ++i)
  {
    // Convert pixel to normalized coordinates
    const double xd = (static_cast<double>(distortedPixels[i].x) - cx) * inv_fx;
    const double yd = (static_cast<double>(distortedPixels[i].y) - cy) * inv_fy;

    double x = 0;
    double y = 0;

    // Remove distortion
    switch (cameraParams.distortion)
    {
      case DistortionModel::none:
        x = xd;
        y = yd;
        break;

      case DistortionModel::plumb_bob:
        distortion::remove_plumb_bob(xd, yd, cameraParams.dist, x, y);
        break;

      case DistortionModel::kannala_brandt:
        distortion::remove_kannala_brandt(xd, yd, cameraParams.dist, x, y);
        break;

      default:
        THROW_EXCEPTION_FMT(
            "Unknown distortion model: %d", static_cast<int>(cameraParams.distortion));
    }

    // Convert back to pixel coordinates
    undistortedPixels[i].x = static_cast<float>(x * fx + cx);
    undistortedPixels[i].y = static_cast<float>(y * fy + cy);
  }

  MRPT_END
}

void mrpt::img::camera_geometry::undistort_points(
    const std::vector<mrpt::img::TPixelCoordf>& distortedPixels,
    std::vector<mrpt::img::TPixelCoordf>& undistortedPixels,
    const mrpt::math::CMatrixDouble33& intrinsicParams,
    const std::vector<double>& distortionParams)
{
  // Create temporary TCamera object
  TCamera cam;
  cam.intrinsicParams = intrinsicParams;
  ASSERT_(distortionParams.size() <= cam.dist.size());
  cam.dist.fill(0);
  for (size_t i = 0; i < distortionParams.size(); ++i)
  {
    cam.dist[i] = distortionParams[i];
  }

  // Use main implementation
  undistort_points(distortedPixels, undistortedPixels, cam);
}

void mrpt::img::camera_geometry::undistort_point(
    const mrpt::img::TPixelCoordf& distortedPt,
    mrpt::img::TPixelCoordf& undistortedPt,
    const mrpt::img::TCamera& cameraParams)
{
  MRPT_START

  const double fx = cameraParams.fx();
  const double fy = cameraParams.fy();
  const double cx = cameraParams.cx();
  const double cy = cameraParams.cy();
  const double inv_fx = 1.0 / fx;
  const double inv_fy = 1.0 / fy;

  // Convert pixel to normalized coordinates
  const double xd = (static_cast<double>(distortedPt.x) - cx) * inv_fx;
  const double yd = (static_cast<double>(distortedPt.y) - cy) * inv_fy;

  double x = 0;
  double y = 0;

  // Remove distortion
  switch (cameraParams.distortion)
  {
    case DistortionModel::none:
      x = xd;
      y = yd;
      break;

    case DistortionModel::plumb_bob:
      distortion::remove_plumb_bob(xd, yd, cameraParams.dist, x, y);
      break;

    case DistortionModel::kannala_brandt:
      distortion::remove_kannala_brandt(xd, yd, cameraParams.dist, x, y);
      break;

    default:
      THROW_EXCEPTION_FMT(
          "Unknown distortion model: %d", static_cast<int>(cameraParams.distortion));
  }

  // Convert back to pixel coordinates
  undistortedPt.x = static_cast<float>(x * fx + cx);
  undistortedPt.y = static_cast<float>(y * fy + cy);

  MRPT_END
}

void mrpt::img::camera_geometry::undistort_points_to_unit_plane(
    const std::vector<mrpt::img::TPixelCoordf>& distortedPixels,
    std::vector<mrpt::math::TPoint2D>& normalizedCoords,
    const mrpt::img::TCamera& cameraParams)
{
  MRPT_START

  const size_t N = distortedPixels.size();
  normalizedCoords.resize(N);

  if (N == 0)
  {
    return;
  }

  const double fx = cameraParams.fx();
  const double fy = cameraParams.fy();
  const double cx = cameraParams.cx();
  const double cy = cameraParams.cy();
  const double inv_fx = 1.0 / fx;
  const double inv_fy = 1.0 / fy;

  for (size_t i = 0; i < N; ++i)
  {
    // Convert pixel to normalized coordinates
    const double xd = (static_cast<double>(distortedPixels[i].x) - cx) * inv_fx;
    const double yd = (static_cast<double>(distortedPixels[i].y) - cy) * inv_fy;

    double x = 0;
    double y = 0;

    // Remove distortion
    switch (cameraParams.distortion)
    {
      case DistortionModel::none:
        x = xd;
        y = yd;
        break;

      case DistortionModel::plumb_bob:
        distortion::remove_plumb_bob(xd, yd, cameraParams.dist, x, y);
        break;

      case DistortionModel::kannala_brandt:
        distortion::remove_kannala_brandt(xd, yd, cameraParams.dist, x, y);
        break;

      default:
        THROW_EXCEPTION_FMT(
            "Unknown distortion model: %d", static_cast<int>(cameraParams.distortion));
    }

    // Return normalized coordinates (unit plane at z=1)
    normalizedCoords[i].x = x;
    normalizedCoords[i].y = y;
  }

  MRPT_END
}