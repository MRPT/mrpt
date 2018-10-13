/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/math/types_math.h>  // Eigen must be included first via MRPT to enable the plugin system

namespace mrpt::vision::pnp
{
/** \addtogroup pnp Perspective-n-Point pose estimation
 *  \ingroup mrpt_vision_grp
 *  @{
 */
/**
 * @class rpnp
 * @author Chandra Mangipudi
 * @date 10/08/16
 * @file rpnp.h
 * @brief Robust - PnP class definition for computing pose
 */
class rpnp
{
	Eigen::MatrixXd
		obj_pts;  //! Object Points (n X 3) in Camera Co-ordinate system
	Eigen::MatrixXd
		img_pts;  //! Image Points (n X 3) in Camera Co-ordinate system
	Eigen::MatrixXd cam_intrinsic;  //! Camera Intrinsic Matrix
	Eigen::MatrixXd P;  //! Transposed Object Points (3 X n) for computations
	Eigen::MatrixXd Q;  //! Transposed Image Points (3 X n) for computations

	Eigen::Matrix3d R;  //! Rotation matrix
	Eigen::Vector3d t;  //! Translation vector
	int n;  //! Number of 2D/3D correspondences

   public:
	//! Constructor for rpnp class
	rpnp(
		Eigen::MatrixXd obj_pts_, Eigen::MatrixXd img_pts_,
		Eigen::MatrixXd cam_, int n0);

	/**
	 * @brief Function to compute pose
	 * @param[out] R_ Rotaiton matrix
	 * @param[out] t_ Translation vector
	 * @return Success flag
	 */
	bool compute_pose(
		Eigen::Ref<Eigen::Matrix3d> R_, Eigen::Ref<Eigen::Vector3d> t_);

	/**
	 * @brief Function to compute pose using P3P
	 * @param[in] l1 Internal parameter for P3P computation
	 * @param[in] l2 Internal parameter for P3P computation
	 * @param[in] A5 Internal parameter for P3P computation
	 * @param[in] C1 Internal parameter for P3P computation
	 * @param[in] C2 Internal parameter for P3P computation
	 * @param[in] D1 Internal parameter for P3P computation
	 * @param[in] D2 Internal parameter for P3P computation
	 * @param[in] D3 Internal parameter for P3P computation
	 * @return Output vector
	 */
	Eigen::VectorXd getp3p(
		double l1, double l2, double A5, double C1, double C2, double D1,
		double D2, double D3);

	/**
	 * @brief Get Polynomial from input vector
	 * @param vin Input vector
	 * @return Output Polynomial co-efficients
	 */
	Eigen::VectorXd getpoly7(const Eigen::VectorXd& vin);

	/**
	 * @brief Function to calculate final pose
	 * @param XXc Object points
	 * @param XXw Image Points
	 * @param R2 Final Rotation matrix
	 * @param t2 Final Translation vector
	 */
	void calcampose(
		Eigen::MatrixXd& XXc, Eigen::MatrixXd& XXw, Eigen::Matrix3d& R2,
		Eigen::Vector3d& t2);
};

/** @}  */  // end of grouping
}  // namespace mrpt::vision::pnp
