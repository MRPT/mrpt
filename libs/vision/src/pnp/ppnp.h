/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#include <iostream>
#include <mrpt/math/types_math.h>  // Eigen must be included first via MRPT to enable the plugin system
#include <Eigen/Dense>

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
		const Eigen::MatrixXd& obj_pts, const Eigen::MatrixXd& img_pts,
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
