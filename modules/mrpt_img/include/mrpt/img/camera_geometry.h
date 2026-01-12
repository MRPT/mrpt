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

#pragma once

#include <mrpt/img/TCamera.h>
#include <mrpt/img/TPixelCoord.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/TPose3DQuat.h>

/** Functions for pinhole camera models: point projection, distortion, and undistortion.
 * \ingroup mrpt_img_grp */
namespace mrpt::img::camera_geometry
{
/** \addtogroup mrpt_img_grp
 * @{ */

// ============================================================================
//  PROJECTION FUNCTIONS (3D -> 2D)
// ============================================================================

/** Project 3D points to image plane without distortion (pinhole model only).
 *
 * This applies only the intrinsic camera matrix transformation:
 * \f[ u = f_x \frac{X}{Z} + c_x, \quad v = f_y \frac{Y}{Z} + c_y \f]
 *
 * \param points3D [IN] 3D points in world coordinates (meters)
 * \param cameraPose [IN] Camera pose in world coordinates
 * \param intrinsicParams [IN] 3x3 camera calibration matrix K
 * \param projectedPoints [OUT] Projected pixel coordinates (resized automatically)
 * \param acceptPointsBehind [IN] If false, points with Z≤0 are marked as (-1,-1)
 *
 * \note Points behind the camera (Z≤0 in camera frame) are marked with
 *       pixel coordinates (-1,-1) unless acceptPointsBehind=true.
 *
 * \sa projectPoints_with_distortion
 */
void projectPoints(
    const std::vector<mrpt::math::TPoint3D>& points3D,
    const mrpt::math::TPose3D& cameraPose,
    const mrpt::math::CMatrixDouble33& intrinsicParams,
    std::vector<mrpt::img::TPixelCoordf>& projectedPoints,
    bool acceptPointsBehind = false);

/** Project 3D points to image plane with lens distortion.
 *
 * Applies the full camera model:
 * 1. Transform point to camera frame: P_cam = cameraPose^{-1} * P_world
 * 2. Normalize: (x,y) = (X/Z, Y/Z)
 * 3. Apply distortion: (x_d, y_d) = distort(x, y)
 * 4. Project to pixels: (u,v) = (f_x*x_d + c_x, f_y*y_d + c_y)
 *
 * \param points3D [IN] 3D points in world coordinates (meters)
 * \param cameraPose [IN] Camera pose in world coordinates
 * \param cameraParams [IN] Complete camera model (intrinsics + distortion)
 * \param projectedPoints [OUT] Projected pixel coordinates (resized automatically)
 * \param acceptPointsBehind [IN] If false, points with Z≤0 are marked as (-1,-1)
 *
 * \sa projectPoint_with_distortion, projectPoints
 */
void projectPoints_with_distortion(
    const std::vector<mrpt::math::TPoint3D>& points3D,
    const mrpt::math::TPose3D& cameraPose,
    const mrpt::img::TCamera& cameraParams,
    std::vector<mrpt::img::TPixelCoordf>& projectedPoints,
    bool acceptPointsBehind = false);

/** \overload Using TPose3DQuat for camera pose */
void projectPoints_with_distortion(
    const std::vector<mrpt::math::TPoint3D>& points3D,
    const mrpt::img::TCamera& cameraParams,
    const mrpt::math::TPose3D& cameraPose,
    std::vector<mrpt::img::TPixelCoordf>& projectedPoints,
    bool acceptPointsBehind = false);

/** Project a single 3D point (in camera frame) to image plane with distortion.
 *
 * \param pointInCamFrame [IN] 3D point in camera coordinate frame (X right, Y down, Z forward)
 * \param cameraParams [IN] Complete camera model (intrinsics + distortion)
 * \param pixel [OUT] Projected pixel coordinates
 * \param acceptPointsBehind [IN] If false, points with Z≤0 are marked as (-1,-1)
 *
 * \sa projectPoints_with_distortion
 */
void projectPoint_with_distortion(
    const mrpt::math::TPoint3D& pointInCamFrame,
    const mrpt::img::TCamera& cameraParams,
    mrpt::img::TPixelCoordf& pixel,
    bool acceptPointsBehind = false);

/** Project a single 3D point without distortion (template version).
 *
 * \tparam INVERSE_CAM_POSE How camera pose F is interpreted:
 *         - false: Point in camera = P ⊖ F (inverse composition, typical use)
 *         - true:  Point in camera = F ⊕ P (direct composition)
 *
 * \param cameraParams [IN] Camera intrinsic parameters
 * \param cameraPose [IN] Camera pose
 * \param point3D [IN] 3D point in world coordinates
 * \return Projected pixel coordinates
 */
template <bool INVERSE_CAM_POSE>
inline mrpt::img::TPixelCoordf projectPoint(
    const mrpt::img::TCamera& cameraParams,
    const mrpt::math::TPose3D& cameraPose,
    const mrpt::math::TPoint3D& point3D)
{
  mrpt::math::TPoint3D pt;
  if (INVERSE_CAM_POSE)
  {
    pt = cameraPose.composePoint(point3D);
  }
  else
  {
    pt = cameraPose.inverseComposePoint(point3D);
  }

  ASSERT_(pt.z != 0);

  return {
      static_cast<float>(cameraParams.cx() + cameraParams.fx() * pt.x / pt.z),
      static_cast<float>(cameraParams.cy() + cameraParams.fy() * pt.y / pt.z)};
}

/** Project a single 3D point (already in camera frame) without distortion.
 *
 * \tparam POINT Any type with .x, .y, .z members
 * \param pointInCamFrame [IN] 3D point in camera coordinate frame
 * \param cameraParams [IN] Camera intrinsic parameters
 * \return Projected pixel coordinates
 */
template <typename POINT>
mrpt::img::TPixelCoordf projectPoint(
    const POINT& pointInCamFrame, const mrpt::img::TCamera& cameraParams)
{
  ASSERT_(pointInCamFrame.z != 0);
  return {
      static_cast<float>(
          cameraParams.cx() + cameraParams.fx() * pointInCamFrame.x / pointInCamFrame.z),
      static_cast<float>(
          cameraParams.cy() + cameraParams.fy() * pointInCamFrame.y / pointInCamFrame.z)};
}

// ============================================================================
//  UNDISTORTION FUNCTIONS (Remove lens distortion)
// ============================================================================

/** Remove lens distortion from pixel coordinates (batch version).
 *
 * Converts distorted pixel coordinates to undistorted pixel coordinates
 * using iterative refinement. The output pixels can be used with the
 * undistorted camera model (distortion=none).
 *
 * \param distortedPixels [IN] Distorted pixel coordinates as captured by camera
 * \param undistortedPixels [OUT] Undistorted pixel coordinates (resized automatically)
 * \param cameraParams [IN] Complete camera model including distortion parameters
 *
 * \sa undistort_point, undistort_points_to_unit_plane
 */
void undistort_points(
    const std::vector<mrpt::img::TPixelCoordf>& distortedPixels,
    std::vector<mrpt::img::TPixelCoordf>& undistortedPixels,
    const mrpt::img::TCamera& cameraParams);

/** \overload Using separate intrinsic matrix and distortion vector
 *
 * \note The distortion model is inferred from the size of distortionParams:
 *       4-5 elements: plumb_bob, 8 elements: plumb_bob with all coefficients
 */
void undistort_points(
    const std::vector<mrpt::img::TPixelCoordf>& distortedPixels,
    std::vector<mrpt::img::TPixelCoordf>& undistortedPixels,
    const mrpt::math::CMatrixDouble33& intrinsicParams,
    const std::vector<double>& distortionParams);

/** Remove lens distortion from a single pixel coordinate.
 *
 * \param distortedPt [IN] Distorted pixel coordinates
 * \param undistortedPt [OUT] Undistorted pixel coordinates
 * \param cameraParams [IN] Complete camera model
 *
 * \sa undistort_points
 */
void undistort_point(
    const mrpt::img::TPixelCoordf& distortedPt,
    mrpt::img::TPixelCoordf& undistortedPt,
    const mrpt::img::TCamera& cameraParams);

/** Convert distorted pixels to normalized image plane coordinates.
 *
 * This is the key function for 3D reconstruction and ray casting. It:
 * 1. Removes lens distortion
 * 2. Converts to normalized coordinates on the z=1 plane
 * 3. Returns coordinates that satisfy: pixel ray direction = (x, y, 1)
 *
 * The output normalized coordinates (x,y) represent the intersection of the
 * pixel's ray with the z=1 plane in the camera coordinate system. For a 3D
 * point P=(X,Y,Z) in camera frame, it projects to normalized coords (X/Z, Y/Z).
 *
 * **Usage example for 3D ray computation:**
 * \code
 * TPoint2D norm = normalizedCoords[i];
 * TPoint3D ray_direction(norm.x, norm.y, 1.0);
 * // Ray from camera origin: P(t) = camera_origin + t * ray_direction
 * \endcode
 *
 * \param distortedPixels [IN] Distorted pixel coordinates from camera
 * \param normalizedCoords [OUT] Normalized coordinates on z=1 plane (resized automatically)
 * \param cameraParams [IN] Complete camera model
 *
 * \note Output are NOT pixel coordinates but normalized 3D ray parameters
 *
 * \sa undistort_points
 */
void undistort_points_to_unit_plane(
    const std::vector<mrpt::img::TPixelCoordf>& distortedPixels,
    std::vector<mrpt::math::TPoint2D>& normalizedCoords,
    const mrpt::img::TCamera& cameraParams);

// ============================================================================
//  LOW-LEVEL DISTORTION FUNCTIONS
// ============================================================================

/** Low-level distortion and undistortion functions operating on normalized coordinates.
 *
 * These functions work with normalized image coordinates (x,y) where a 3D point
 * (X,Y,Z) in camera frame projects to (x,y) = (X/Z, Y/Z).
 *
 * **Use these functions when:**
 * - Implementing custom projection pipelines
 * - Testing distortion models in isolation
 * - Porting code from other computer vision libraries
 * - Need fine-grained control over the distortion process
 *
 * **For typical use cases, prefer the high-level functions above.**
 */
namespace distortion
{
/** Apply plumb_bob (radial-tangential) distortion to normalized coordinates.
 *
 * Implements the Brown-Conrady distortion model:
 * \f[
 * \begin{aligned}
 * r^2 &= x^2 + y^2 \\
 * \text{radial} &= \frac{1 + k_1 r^2 + k_2 r^4 + k_3 r^6}{1 + k_4 r^2 + k_5 r^4 + k_6 r^6} \\
 * x_d &= x \cdot \text{radial} + 2p_1 xy + p_2(r^2 + 2x^2) \\
 * y_d &= y \cdot \text{radial} + p_1(r^2 + 2y^2) + 2p_2 xy
 * \end{aligned}
 * \f]
 *
 * \param x [IN] Normalized x coordinate (X/Z)
 * \param y [IN] Normalized y coordinate (Y/Z)
 * \param dist [IN] Distortion coefficients [k1,k2,p1,p2,k3,k4,k5,k6]
 * \param xd [OUT] Distorted x coordinate
 * \param yd [OUT] Distorted y coordinate
 *
 * \sa remove_plumb_bob, apply_kannala_brandt
 */
void apply_plumb_bob(double x, double y, const std::array<double, 8>& dist, double& xd, double& yd);

/** Apply Kannala-Brandt fish-eye distortion to normalized coordinates.
 *
 * Implements the model from "A Generic Camera Model and Calibration Method
 * for Conventional, Wide-Angle, and Fish-Eye Lenses" (Kannala & Brandt, 2006):
 * \f[
 * \begin{aligned}
 * r &= \sqrt{x^2 + y^2} \\
 * \theta &= \arctan(r) \\
 * \theta_d &= \theta(1 + k_1\theta^2 + k_2\theta^4 + k_3\theta^6 + k_4\theta^8) \\
 * x_d &= \frac{\theta_d}{r} x, \quad y_d = \frac{\theta_d}{r} y
 * \end{aligned}
 * \f]
 *
 * \param x [IN] Normalized x coordinate
 * \param y [IN] Normalized y coordinate
 * \param dist [IN] Distortion coefficients [k1,k2,_,_,k3,k4,_,_] (indices 2,3,6,7 unused)
 * \param xd [OUT] Distorted x coordinate
 * \param yd [OUT] Distorted y coordinate
 *
 * \sa remove_kannala_brandt, apply_plumb_bob
 */
void apply_kannala_brandt(
    double x, double y, const std::array<double, 8>& dist, double& xd, double& yd);

/** Remove plumb_bob distortion using iterative refinement.
 *
 * Given distorted normalized coordinates (x_d, y_d), finds undistorted
 * coordinates (x, y) such that apply_plumb_bob(x,y) ≈ (x_d, y_d).
 *
 * Uses fixed-point iteration:
 * 1. Start with x = x_d, y = y_d
 * 2. Compute expected distortion at (x,y)
 * 3. Update: x_new = (x_d - tangential_x) / radial_factor
 * 4. Repeat until convergence or max iterations
 *
 * \param xd [IN] Distorted x coordinate
 * \param yd [IN] Distorted y coordinate
 * \param dist [IN] Distortion coefficients [k1,k2,p1,p2,k3,k4,k5,k6]
 * \param x [OUT] Undistorted x coordinate
 * \param y [OUT] Undistorted y coordinate
 * \param iterations [IN] Number of refinement iterations (default: 5, typically sufficient)
 *
 * \note 5 iterations typically achieve sub-pixel accuracy for typical distortion magnitudes
 *
 * \sa apply_plumb_bob, remove_kannala_brandt
 */
void remove_plumb_bob(
    double xd,
    double yd,
    const std::array<double, 8>& dist,
    double& x,
    double& y,
    int iterations = 5);

/** Remove Kannala-Brandt fish-eye distortion using Newton-Raphson method.
 *
 * Solves the nonlinear equation to invert the fish-eye model:
 * \f[ \theta_d = \theta(1 + k_1\theta^2 + k_2\theta^4 + k_3\theta^6 + k_4\theta^8) \f]
 *
 * Uses Newton-Raphson iteration:
 * \f[ \theta_{n+1} = \theta_n - \frac{f(\theta_n)}{f'(\theta_n)} \f]
 * where \f$f(\theta) = \theta \cdot \text{poly}(\theta) - \theta_d\f$
 *
 * \param xd [IN] Distorted x coordinate
 * \param yd [IN] Distorted y coordinate
 * \param dist [IN] Distortion coefficients [k1,k2,_,_,k3,k4,_,_]
 * \param x [OUT] Undistorted x coordinate
 * \param y [OUT] Undistorted y coordinate
 * \param iterations [IN] Number of Newton-Raphson iterations (default: 10)
 *
 * \note Fish-eye models require more iterations than plumb_bob due to higher nonlinearity
 *
 * \sa apply_kannala_brandt, remove_plumb_bob
 */
void remove_kannala_brandt(
    double xd,
    double yd,
    const std::array<double, 8>& dist,
    double& x,
    double& y,
    int iterations = 10);

}  // namespace distortion

/** @} */  // end of grouping

}  // namespace mrpt::img::camera_geometry