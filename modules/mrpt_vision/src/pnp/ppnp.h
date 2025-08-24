/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2025, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/
//#include <mrpt/math/types_math.h>  // Eigen must be included first via MRPT to
// enable the plugin system
#include <Eigen/Dense>
#include <iostream>

namespace mrpt::vision::pnp
{
/** \addtogroup pnp Perspective-n-Point pose estimation
 *  \ingroup mrpt_vision_grp
 *  @{
 */

/**
 * @class ppnp
 * @author Chandra Mangipudi
 * @date 10/08/16
 * @file ppnp.h
 * @brief Procrustes - PnP
 */
class ppnp
{
 public:
  //! Constructor for the P-PnP class
  ppnp(
      const Eigen::MatrixXd& obj_pts,
      const Eigen::MatrixXd& img_pts,
      const Eigen::MatrixXd& cam_intrinsic);

  /**
   * @brief Function to compute pose
   * @param[out] R Rotation matrix
   * @param t Trnaslation Vector
   * @param n Number of 2d/3d correspondences
   * @return
   */
  bool compute_pose(Eigen::Matrix3d& R, Eigen::Vector3d& t, int n);

 private:
  Eigen::MatrixXd P;  //! Image points in pixels
  Eigen::MatrixXd S;  //! Object points in Camera Co-ordinate system
  Eigen::MatrixXd C;  //! Camera intrinsic matrix
};

/** @}  */  // end of grouping
}  // namespace mrpt::vision::pnp
