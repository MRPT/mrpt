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

#include <mrpt/img/CUndistortMap.h>
#include <mrpt/img/camera_geometry.h>

#include "remap_bilinear.h"

using namespace mrpt;
using namespace mrpt::img;

void CUndistortMap::setFromCamParams(const mrpt::img::TCamera& campar)
{
  MRPT_START

  m_camera_params = campar;

  const uint32_t ncols = campar.ncols;
  const uint32_t nrows = campar.nrows;

  m_dat_mapx.resize(static_cast<size_t>(nrows) * ncols);
  m_dat_mapy.resize(static_cast<size_t>(nrows) * ncols);

  const double fx = campar.fx();
  const double fy = campar.fy();
  const double cx = campar.cx();
  const double cy = campar.cy();
  const double inv_fx = 1.0 / fx;
  const double inv_fy = 1.0 / fy;

  // For each output (undistorted) pixel, compute the corresponding
  // source (distorted) pixel using the forward distortion model.
  for (uint32_t v = 0; v < nrows; ++v)
  {
    const size_t row_off = static_cast<size_t>(v) * ncols;
    for (uint32_t u = 0; u < ncols; ++u)
    {
      // Normalize output pixel to camera coordinates
      const double xn = (static_cast<double>(u) - cx) * inv_fx;
      const double yn = (static_cast<double>(v) - cy) * inv_fy;

      double xd = xn;
      double yd = yn;

      // Apply forward distortion to get the source (distorted) normalized coords
      switch (campar.distortion)
      {
        case DistortionModel::none:
          break;
        case DistortionModel::plumb_bob:
          camera_geometry::distortion::apply_plumb_bob(xn, yn, campar.dist, xd, yd);
          break;
        case DistortionModel::kannala_brandt:
          camera_geometry::distortion::apply_kannala_brandt(xn, yn, campar.dist, xd, yd);
          break;
        default:
          THROW_EXCEPTION_FMT(
              "Unknown distortion model: %d", static_cast<int>(campar.distortion));
      }

      // Convert back to pixel coordinates in the distorted (source) image
      m_dat_mapx[row_off + u] = static_cast<float>(xd * fx + cx);
      m_dat_mapy[row_off + u] = static_cast<float>(yd * fy + cy);
    }
  }

  MRPT_END
}

void CUndistortMap::undistort(const mrpt::img::CImage& in_img, mrpt::img::CImage& out_img) const
{
  MRPT_START
  if (m_dat_mapx.empty())
  {
    THROW_EXCEPTION("Error: setFromCamParams() must be called prior to undistort().");
  }

  detail::remap_bilinear(
      in_img, out_img, m_dat_mapx.data(), m_dat_mapy.data(),
      static_cast<int>(m_camera_params.ncols), static_cast<int>(m_camera_params.nrows));

  MRPT_END
}

void CUndistortMap::undistort(mrpt::img::CImage& in_out_img) const
{
  MRPT_START
  if (m_dat_mapx.empty())
  {
    THROW_EXCEPTION("Error: setFromCamParams() must be called prior to undistort().");
  }

  CImage tmp = in_out_img.makeDeepCopy();
  detail::remap_bilinear(
      tmp, in_out_img, m_dat_mapx.data(), m_dat_mapy.data(),
      static_cast<int>(m_camera_params.ncols), static_cast<int>(m_camera_params.nrows));

  MRPT_END
}
