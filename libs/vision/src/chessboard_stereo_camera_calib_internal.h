/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/math/geometry.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/vision/chessboard_camera_calib.h>
#include <Eigen/Dense>

namespace mrpt::vision
{
// State of the Lev-Marq optimization:
struct lm_stat_t
{
	const TCalibrationStereoImageList& images;
	const std::vector<size_t>& valid_image_pair_indices;
	const std::vector<mrpt::math::TPoint3D>& obj_points;

	// State being optimized:
	//  N*left_cam_pose + right2left_pose + left_cam_params + right_cam_params
	// Poses of the origin of coordinates of the pattern wrt the left camera
	std::vector<mrpt::math::TPose3D> left_cam_poses;
	mrpt::math::TPose3D right2left_pose;
	/** [fx fy cx cy k1 k2 k3 t1 t2] */
	mrpt::math::CVectorFixedDouble<9> left_cam_params, right_cam_params;

	// Ctor
	lm_stat_t(
		const TCalibrationStereoImageList& _images,
		const std::vector<size_t>& _valid_image_pair_indices,
		const std::vector<mrpt::math::TPoint3D>& _obj_points)
		: images(_images),
		  valid_image_pair_indices(_valid_image_pair_indices),
		  obj_points(_obj_points)
	{
		// Initial
		left_cam_poses.assign(
			images.size(), mrpt::math::TPose3D(0, 0, 1, 0, 0, 0));
	}

	// Swap:
	void swap(lm_stat_t& o)
	{
		left_cam_poses.swap(o.left_cam_poses);
		std::swap(right2left_pose, o.right2left_pose);
		std::swap(left_cam_params, o.left_cam_params);
		std::swap(right_cam_params, o.right_cam_params);
	}
};

/** Data associated to *each observation* in the Lev-Marq. model */
struct TResidJacobElement
{
	/** [u_l v_l  u_r v_r]: left/right camera pixels */
	Eigen::Matrix<double, 4, 1> predicted_obs;
	/**  = predicted_obs - observations */
	Eigen::Matrix<double, 4, 1> residual;
	/** Jacobian. 4=the two predicted pixels; 30=Read below for the meaning of
	 * these 30 variables */
	Eigen::Matrix<double, 4, 30> J;
};

using TResidualJacobianList = std::vector<std::vector<TResidJacobElement>>;

// Auxiliary functions for the Lev-Marq algorithm:
double recompute_errors_and_Jacobians(
	const lm_stat_t& lm_stat, TResidualJacobianList& res_jac,
	bool use_robust_kernel, double kernel_param);
void build_linear_system(
	const TResidualJacobianList& res_jac, const std::vector<size_t>& var_indxs,
	mrpt::math::CVectorDynamic<double>& minus_g, mrpt::math::CMatrixDouble& H);
void add_lm_increment(
	const mrpt::math::CVectorDynamic<double>& eps,
	const std::vector<size_t>& var_indxs, lm_stat_t& new_lm_stat);
}  // namespace mrpt::vision
