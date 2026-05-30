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

#include <mrpt/img/CStereoRectifyMap.h>
#include <mrpt/img/camera_geometry.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>

#include "remap_bilinear.h"

using namespace mrpt;
using namespace mrpt::img;
using namespace mrpt::math;

// ============================================================================
//  Anonymous namespace: helper functions
// ============================================================================
namespace
{
/** Build an undistort+rectify remap for a single camera.
 *  For each output (rectified) pixel, compute the corresponding
 *  source (original distorted) pixel.
 *
 *  \param cam       Original camera intrinsics + distortion
 *  \param R_rect    3x3 rectification rotation (maps rectified camera -> original camera)
 *  \param P_new     3x4 new projection matrix [K_new | 0] (only 3x3 part used)
 *  \param out_w     Output width
 *  \param out_h     Output height
 *  \param mapx      [out] float source x coordinates (out_h * out_w)
 *  \param mapy      [out] float source y coordinates (out_h * out_w)
 */
void build_rectify_map(
    const mrpt::img::TCamera& cam,
    const Eigen::Matrix3d& R_rect_inv,
    const Eigen::Matrix3d& K_new,
    int out_w,
    int out_h,
    std::vector<float>& mapx,
    std::vector<float>& mapy)
{
  const size_t npix = static_cast<size_t>(out_w) * out_h;
  mapx.resize(npix);
  mapy.resize(npix);

  const Eigen::Matrix3d K_new_inv = K_new.inverse();

  const double fx = cam.fx();
  const double fy = cam.fy();
  const double cx = cam.cx();
  const double cy = cam.cy();

  for (int v = 0; v < out_h; ++v)
  {
    const size_t row_off = static_cast<size_t>(v) * out_w;
    for (int u = 0; u < out_w; ++u)
    {
      // 1. Convert output pixel to homogeneous coords and un-project
      //    through new intrinsics
      Eigen::Vector3d p_rect(u, v, 1.0);
      Eigen::Vector3d p_norm = K_new_inv * p_rect;

      // 2. Apply inverse rectification rotation to get original camera
      //    normalized coords
      Eigen::Vector3d p_cam = R_rect_inv * p_norm;

      // Normalize to z=1
      const double xn = p_cam.x() / p_cam.z();
      const double yn = p_cam.y() / p_cam.z();

      // 3. Apply original distortion
      double xd = xn;
      double yd = yn;

      switch (cam.distortion)
      {
        case DistortionModel::none:
          break;
        case DistortionModel::plumb_bob:
          camera_geometry::distortion::apply_plumb_bob(xn, yn, cam.dist, xd, yd);
          break;
        case DistortionModel::kannala_brandt:
          camera_geometry::distortion::apply_kannala_brandt(xn, yn, cam.dist, xd, yd);
          break;
        default:
          THROW_EXCEPTION_FMT("Unknown distortion model: %d", static_cast<int>(cam.distortion));
      }

      // 4. Project to original pixel coordinates
      mapx[row_off + u] = static_cast<float>(xd * fx + cx);
      mapy[row_off + u] = static_cast<float>(yd * fy + cy);
    }
  }
}

/** Convert an Eigen 3x3 rotation matrix to an MRPT CQuaternionDouble */
CQuaternionDouble eigenRotToQuaternion(const Eigen::Matrix3d& R)
{
  Eigen::Quaterniond eq(R);
  eq.normalize();
  // CQuaternionDouble uses (r, x, y, z) order = (w, x, y, z)
  CQuaternionDouble q(eq.w(), eq.x(), eq.y(), eq.z());
  return q;
}

}  // anonymous namespace

// ============================================================================
//  CStereoRectifyMap implementation
// ============================================================================

void CStereoRectifyMap::internal_invalidate()
{
  m_dat_mapx_left.clear();
  m_dat_mapx_right.clear();
  m_dat_mapy_left.clear();
  m_dat_mapy_right.clear();
}

void CStereoRectifyMap::setAlpha(double alpha)
{
  m_alpha = alpha;
  this->internal_invalidate();
}

void CStereoRectifyMap::enableResizeOutput(
    bool enable, unsigned int target_width, unsigned int target_height)
{
  m_resize_output = enable;
  m_resize_output_value.x = target_width;
  m_resize_output_value.y = target_height;
  this->internal_invalidate();
}

void CStereoRectifyMap::enableBothCentersCoincide(bool enable)
{
  m_enable_both_centers_coincide = enable;
  this->internal_invalidate();
}

void CStereoRectifyMap::setFromCamParams(const mrpt::img::TStereoCamera& params)
{
  MRPT_START

  const mrpt::img::TCamera& cam1 = params.leftCamera;
  const mrpt::img::TCamera& cam2 = params.rightCamera;

  ASSERT_(cam1.ncols == cam2.ncols && cam1.nrows == cam2.nrows);

  const uint32_t ncols = cam1.ncols;
  const uint32_t nrows = cam1.nrows;

  const uint32_t ncols_out = m_resize_output ? m_resize_output_value.x : ncols;
  const uint32_t nrows_out = m_resize_output ? m_resize_output_value.y : nrows;

  m_camera_params = params;

  // -----------------------------------------------------------------------
  // 1. Extract relative pose: R, T from right camera w.r.t. left camera
  // -----------------------------------------------------------------------
  // rightCameraPose is TPose3DQuat (x, y, z, qr, qx, qy, qz)
  const auto& rp = params.rightCameraPose;

  // Convert quaternion to Eigen rotation matrix
  // Note: CQuaternion uses (r, x, y, z) = (w, x, y, z)
  Eigen::Quaterniond eq_fwd(rp.qr, rp.qx, rp.qy, rp.qz);
  eq_fwd.normalize();
  Eigen::Matrix3d R_fwd = eq_fwd.toRotationMatrix();
  Eigen::Vector3d T_fwd(rp.x, rp.y, rp.z);

  // We need the INVERSE pose (OpenCV convention): the transform that takes
  // points from right camera frame to left camera frame.
  Eigen::Matrix3d R = R_fwd.transpose();
  Eigen::Vector3d T = -R * T_fwd;

  // -----------------------------------------------------------------------
  // 2. Bouguet-style stereo rectification
  //    Split rotation equally between cameras and align epipolar lines
  //    horizontally.
  // -----------------------------------------------------------------------

  // 2a. Compute the half-rotation: rotate each camera halfway toward the other
  Eigen::AngleAxisd aa(R);
  Eigen::Matrix3d R_half = Eigen::AngleAxisd(aa.angle() * 0.5, aa.axis()).toRotationMatrix();

  // R1 = R_half     (rotate left camera forward by half)
  // R2 = R_half * R^T  (rotate right camera backward by half)
  // After this, both cameras have the same orientation.

  // 2b. Compute the rectification rotation to make epipolar lines horizontal.
  // The baseline in the half-rotated frame:
  Eigen::Vector3d T_half = R_half * T;

  // e1 = baseline direction (should become the new x-axis)
  Eigen::Vector3d e1 = T_half.normalized();

  // e2 = perpendicular to e1 and the old y-axis (or z if degenerate)
  Eigen::Vector3d up(0, -1, 0);  // camera y points down, so "up" is -y
  if (std::abs(e1.dot(up)) > 0.9)
  {
    up = Eigen::Vector3d(0, 0, 1);
  }
  Eigen::Vector3d e2 = (e1.cross(up)).normalized();
  // If e1 is parallel to up, try a different fallback axis
  if (!e2.allFinite() || e2.norm() < 0.5)
  {
    up = Eigen::Vector3d(1, 0, 0);
    e2 = (e1.cross(up)).normalized();
  }

  // e3 = e1 x e2
  Eigen::Vector3d e3 = e1.cross(e2);

  // Rrect: rotation that aligns the baseline with the x-axis
  Eigen::Matrix3d Rrect;
  Rrect.row(0) = e1.transpose();
  Rrect.row(1) = e2.transpose();
  Rrect.row(2) = e3.transpose();

  // Final rectification rotations:
  Eigen::Matrix3d R1 = Rrect * R_half;
  Eigen::Matrix3d R2 = Rrect * R_half * R.transpose();

  // -----------------------------------------------------------------------
  // 3. Compute new intrinsic matrix K_new
  // -----------------------------------------------------------------------
  // Use average focal length from both cameras
  const double f_new = (cam1.fx() + cam1.fy() + cam2.fx() + cam2.fy()) * 0.25;

  // New principal point: center of output image by default
  double cx_new = static_cast<double>(ncols_out - 1) * 0.5;
  double cy_new = static_cast<double>(nrows_out - 1) * 0.5;

  if (m_enable_both_centers_coincide)
  {
    // Average the original principal points (mapped through rectification)
    cx_new = (cam1.cx() + cam2.cx()) * 0.5;
    cy_new = (cam1.cy() + cam2.cy()) * 0.5;
    if (m_resize_output)
    {
      cx_new *= static_cast<double>(ncols_out) / ncols;
      cy_new *= static_cast<double>(nrows_out) / nrows;
    }
  }

  // Handle alpha parameter for ROI adjustment
  // alpha = 0: zoom in, only valid pixels
  // alpha = 1: zoom out, all original pixels, black borders
  // alpha = -1 (default): no adjustment
  if (m_alpha >= 0.0 && m_alpha <= 1.0)
  {
    // Scale focal length based on alpha: smaller alpha -> larger f (more zoom)
    // This is a simplified version of OpenCV's getOptimalNewCameraMatrix
    const double scale = 1.0 + m_alpha * 0.2;  // Heuristic scaling
    // f_new stays as is for now; full implementation would compute inner/outer
    // rectangles of the undistorted image. For basic version, we keep average f.
    (void)scale;
  }

  Eigen::Matrix3d K_new = Eigen::Matrix3d::Zero();
  K_new(0, 0) = f_new;
  K_new(1, 1) = f_new;
  K_new(0, 2) = cx_new;
  K_new(1, 2) = cy_new;
  K_new(2, 2) = 1.0;

  // -----------------------------------------------------------------------
  // 4. Build remap tables
  // -----------------------------------------------------------------------
  // The inverse rectification rotations: for each rectified pixel, we need
  // to find the original pixel by rotating back.
  Eigen::Matrix3d R1_inv = R1.transpose();
  Eigen::Matrix3d R2_inv = R2.transpose();

  build_rectify_map(
      cam1, R1_inv, K_new, static_cast<int>(ncols_out), static_cast<int>(nrows_out),
      m_dat_mapx_left, m_dat_mapy_left);

  build_rectify_map(
      cam2, R2_inv, K_new, static_cast<int>(ncols_out), static_cast<int>(nrows_out),
      m_dat_mapx_right, m_dat_mapy_right);

  // -----------------------------------------------------------------------
  // 5. Populate rectified image parameters
  // -----------------------------------------------------------------------
  m_rectified_image_params.leftCamera.ncols = ncols_out;
  m_rectified_image_params.leftCamera.nrows = nrows_out;
  m_rectified_image_params.leftCamera.setIntrinsicParamsFromValues(f_new, f_new, cx_new, cy_new);
  m_rectified_image_params.leftCamera.dist.fill(0);
  m_rectified_image_params.leftCamera.distortion = DistortionModel::none;
  m_rectified_image_params.leftCamera.focalLengthMeters = cam1.focalLengthMeters;

  m_rectified_image_params.rightCamera.ncols = ncols_out;
  m_rectified_image_params.rightCamera.nrows = nrows_out;
  m_rectified_image_params.rightCamera.setIntrinsicParamsFromValues(f_new, f_new, cx_new, cy_new);
  m_rectified_image_params.rightCamera.dist.fill(0);
  m_rectified_image_params.rightCamera.distortion = DistortionModel::none;
  m_rectified_image_params.rightCamera.focalLengthMeters = cam2.focalLengthMeters;

  m_rectified_image_params.rightCameraPose = params.rightCameraPose;

  // -----------------------------------------------------------------------
  // 6. Store rectification rotations as quaternions
  // -----------------------------------------------------------------------
  m_rot_left = eigenRotToQuaternion(R1);
  m_rot_right = eigenRotToQuaternion(R2);

  MRPT_END
}

void CStereoRectifyMap::rectify(
    const mrpt::img::CImage& in_left_image,
    const mrpt::img::CImage& in_right_image,
    mrpt::img::CImage& out_left_image,
    mrpt::img::CImage& out_right_image) const
{
  MRPT_START

  if (!isSet())
  {
    THROW_EXCEPTION("Error: setFromCamParams() must be called prior to rectify().");
  }

  ASSERTMSG_(
      &in_left_image != &out_left_image && &in_right_image != &out_right_image,
      "In-place rectify not supported");

  const int ncols_out = m_resize_output ? m_resize_output_value.x
                                        : static_cast<int>(m_camera_params.leftCamera.ncols);
  const int nrows_out = m_resize_output ? m_resize_output_value.y
                                        : static_cast<int>(m_camera_params.leftCamera.nrows);

  detail::remap_bilinear(
      in_left_image, out_left_image, m_dat_mapx_left.data(), m_dat_mapy_left.data(), ncols_out,
      nrows_out);

  detail::remap_bilinear(
      in_right_image, out_right_image, m_dat_mapx_right.data(), m_dat_mapy_right.data(), ncols_out,
      nrows_out);

  MRPT_END
}

const mrpt::img::TStereoCamera& CStereoRectifyMap::getRectifiedImageParams() const
{
  ASSERTMSG_(isSet(), "Error: setFromCamParams() must be called before.");
  return m_rectified_image_params;
}

const mrpt::img::TCamera& CStereoRectifyMap::getRectifiedLeftImageParams() const
{
  ASSERTMSG_(isSet(), "Error: setFromCamParams() must be called before.");
  return m_rectified_image_params.leftCamera;
}

const mrpt::img::TCamera& CStereoRectifyMap::getRectifiedRightImageParams() const
{
  ASSERTMSG_(isSet(), "Error: setFromCamParams() must be called before.");
  return m_rectified_image_params.rightCamera;
}

void CStereoRectifyMap::setRectifyMaps(
    const std::vector<float>& left_x,
    const std::vector<float>& left_y,
    const std::vector<float>& right_x,
    const std::vector<float>& right_y)
{
  m_dat_mapx_left = left_x;
  m_dat_mapy_left = left_y;
  m_dat_mapx_right = right_x;
  m_dat_mapy_right = right_y;
}

void CStereoRectifyMap::setRectifyMapsFast(
    std::vector<float>& left_x,
    std::vector<float>& left_y,
    std::vector<float>& right_x,
    std::vector<float>& right_y)
{
  left_x.swap(m_dat_mapx_left);
  left_y.swap(m_dat_mapy_left);
  right_x.swap(m_dat_mapx_right);
  right_y.swap(m_dat_mapy_right);
}
