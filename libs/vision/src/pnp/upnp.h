/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
// M*//////////////////////////////////////////////////////////////////////////////////////
//
// IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
// By downloading, copying, installing or using the software you agree to this
// license.
// If you do not agree to this license, do not download, install,
// copy or use the software.
//
//
// License Agreement
// For Open Source Computer Vision Library
//
// Copyright (C) 2000, Intel Corporation, all rights reserved.
// Copyright (C) 2013, OpenCV Foundation, all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without
// modification,
// are permitted provided that the following conditions are met:
//
// * Redistribution's of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// * Redistribution's in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// * The name of the copyright holders may not be used to endorse or promote
// products
// derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is"
// and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are
// disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any
// direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
// M*/

/****************************************************************************************\
* Exhaustive Linearization for Robust Camera Pose and Focal Length Estimation.
* Contributed by Edgar Riba
\****************************************************************************************/

#pragma once

#include <mrpt/config.h>
#include <mrpt/otherlibs/do_opencv_includes.h>

#if MRPT_HAS_OPENCV
//#include <opencv2/core/core_c.h>
#include <iostream>

namespace mrpt::vision::pnp
{
/** \addtogroup pnp Perspective-n-Point pose estimation
 *  \ingroup mrpt_vision_grp
 *  @{
 */

/**
 * @class upnp
 * @author Chandra Mangipudi
 * @date 12/08/16
 * @file upnp.h
 * @brief Unified PnP - Eigen Wrapper for OpenCV function
 */
class upnp
{
   public:
	//! Constructor for UPnP class
	upnp(
		const cv::Mat& cameraMatrix, const cv::Mat& opoints,
		const cv::Mat& ipoints);

	//! Destructor for UPnP class
	~upnp();

	/**
	 * @brief Function to compute pose
	 * @param[out] R Rotation Matrix
	 * @param[out] t Translation Vector
	 * @return
	 */
	double compute_pose(cv::Mat& R, cv::Mat& t);

   private:
	/**
	 * @brief Initialize camera variables using camera intrinsic matrix
	 * @param[in] cameraMatrix Camera Intrinsic Matrix
	 */
	template <typename T>
	void init_camera_parameters(const cv::Mat& cameraMatrix)
	{
		uc = cameraMatrix.at<T>(0, 2);
		vc = cameraMatrix.at<T>(1, 2);
		fu = 1;
		fv = 1;
	}

	/**
	 * @brief Iniialize Object points and image points from OpenCV Matrix
	 * @param[in] opoints Object Points
	 * @param[in] ipoints Image Points
	 */
	template <typename OpointType, typename IpointType>
	void init_points(const cv::Mat& opoints, const cv::Mat& ipoints)
	{
		for (int i = 0; i < number_of_correspondences; i++)
		{
			pws[3 * i] = opoints.at<OpointType>(i).x;
			pws[3 * i + 1] = opoints.at<OpointType>(i).y;
			pws[3 * i + 2] = opoints.at<OpointType>(i).z;

			us[2 * i] = ipoints.at<IpointType>(i).x;
			us[2 * i + 1] = ipoints.at<IpointType>(i).y;
		}
	}

	/**
	 * @brief Compute the reprojection error using the estimated Rotation matrix
	 * and Translation Vector
	 * @param[in] R Rotation matrix
	 * @param[in] t Trnaslation Vector
	 * @return  Linear least squares error in pixels
	 */
	double reprojection_error(const double R[3][3], const double t[3]);

	/**
	 * @brief Function to select 4 control points
	 */
	void choose_control_points();

	/**
	 * @brief Function to comput @alphas
	 */
	void compute_alphas();

	/**
	 * @brief Function to compute Maucaulay matrix M
	 * @param[out] M Maucaulay matrix
	 * @param[in] row Internal member
	 * @param[in] alphas Internal member
	 * @param[in] u Image pixel x co-ordinate
	 * @param[in] v Image pixel y co-ordinate
	 */
	void fill_M(
		cv::Mat* M, const int row, const double* alphas, const double u,
		const double v);

	/**
	 * @brief Compute the control points
	 * @param[in] betas Internal member variable
	 * @param[in] ut Internal member variable
	 */
	void compute_ccs(const double* betas, const double* ut);

	/**
	 * @brief Compute object points based on control points
	 */
	void compute_pcs();

	/**
	 * @brief Internal member function
	 */
	void solve_for_sign();

	/**
	 * @brief Function to approximately calculate betas and focal length
	 * @param[in] Ut
	 * @param[in] Rho
	 * @param[out] betas
	 * @param[out] efs
	 */
	void find_betas_and_focal_approx_1(
		cv::Mat* Ut, cv::Mat* Rho, double* betas, double* efs);

	/**
	 * @brief Function to calculate betas and focal length (more accurate)
	 * @param[in] Ut
	 * @param[in] Rho
	 * @param[out] betas
	 * @param[out] efs
	 */
	void find_betas_and_focal_approx_2(
		cv::Mat* Ut, cv::Mat* Rho, double* betas, double* efs);

	/**
	 * @brief Function to do a QR decomposition
	 * @param[in] A Matrix to be decomposed
	 * @param[out] b
	 * @param[out] X
	 */
	void qr_solve(cv::Mat* A, cv::Mat* b, cv::Mat* X);

	/**
	 * @brief Internal function
	 * @param[in] M1
	 * @return Distance
	 */
	cv::Mat compute_constraint_distance_2param_6eq_2unk_f_unk(
		const cv::Mat& M1);

	/**
	 * @brief Internal function
	 * @param[in] M1
	 * @param[in] M2
	 * @return Distance
	 */
	cv::Mat compute_constraint_distance_3param_6eq_6unk_f_unk(
		const cv::Mat& M1, const cv::Mat& M2);

	/**
	 * @brief Get all possible solutions
	 * @param[in] betas
	 * @param[out] solutions
	 */
	void generate_all_possible_solutions_for_f_unk(
		const double betas[5], double solutions[18][3]);

	/**
	 * @brief Return the sign of the scalar
	 * @param[in] v
	 * @return True if positive else Flase
	 */
	double sign(const double v);

	/**
	 * @brief Compute the dot product between two vectors
	 * @param[in] v1
	 * @param[in] v2
	 * @return Dot product
	 */
	double dot(const double* v1, const double* v2);

	/**
	 * @brief Compute dot product in 2D with only x and y components
	 * @param[in] v1
	 * @param[in] v2
	 * @return Dot product
	 */
	double dotXY(const double* v1, const double* v2);

	/**
	 * @brief Compute the dot product using only z component
	 * @param[in] v1
	 * @param[in] v2
	 * @return Dot product
	 */
	double dotZ(const double* v1, const double* v2);

	/**
	 * @brief Compute the euclidean distance squared between two points in 3D
	 * @param[in] p1
	 * @param[in] p2
	 * @return Distance squared
	 */
	double dist2(const double* p1, const double* p2);

	/**
	 * @brief Internal fucntion
	 * @param[out] rho
	 */
	void compute_rho(double* rho);

	/**
	 * @brief Internal function
	 * @param[in] ut
	 * @param[out] l_6x12
	 */
	void compute_L_6x12(const double* ut, double* l_6x12);

	/**
	 * @brief Gauss Newton Iterative optimization
	 * @param[in] L_6x12
	 * @param[in] Rho
	 * @param[in,out] current_betas
	 * @param[out] efs
	 */
	void gauss_newton(
		const cv::Mat* L_6x12, const cv::Mat* Rho, double current_betas[4],
		double* efs);

	/**
	 * @brief Compute matrix A and vector b
	 * @param[in] l_6x12
	 * @param[in] rho
	 * @param[in] cb
	 * @param[out] A
	 * @param[out] b
	 * @param[in] f
	 */
	void compute_A_and_b_gauss_newton(
		const double* l_6x12, const double* rho, const double cb[4], cv::Mat* A,
		cv::Mat* b, double const f);

	/**
	 * @brief Function to compute the pose
	 * @param[in] ut
	 * @param[in] betas
	 * @param[out] R Rotation Matrix
	 * @param[out] t Translation VectorS
	 * @return
	 */
	double compute_R_and_t(
		const double* ut, const double* betas, double R[3][3], double t[3]);

	/**
	 * @brief Helper function to function compute_R_and_t()
	 * @param[out] R Rotaiton matrix
	 * @param[out] t Translation vector
	 */
	void estimate_R_and_t(double R[3][3], double t[3]);

	/**
	 * @brief Function to copy the pose
	 * @param[in] R_dst
	 * @param[in] t_dst
	 * @param[out] R_src
	 * @param[out] t_src
	 */
	void copy_R_and_t(
		const double R_dst[3][3], const double t_dst[3], double R_src[3][3],
		double t_src[3]);

	double uc;  //! Image center in x-direction
	double vc;  //! Image center in y-direction
	double fu;  //! Focal length in x-direction
	double fv;  //! Focal length in y-direction

	std::vector<double> pws;  //! Object points
	std::vector<double> us;  //! Image points
	std::vector<double> alphas, pcs;  //! Internal variable
	int number_of_correspondences;  //! Number of 2d/3d correspondences

	double cws[4][3], ccs[4][3];  //! Control point variables
	int max_nr;  //! Internal variable
	double *A1, *A2;  //! Internal variable
};

/** @}  */  // end of grouping
}  // namespace mrpt::vision::pnp
#endif  // Check for OPENCV_LIB
