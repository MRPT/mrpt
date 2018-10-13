/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <vector>
#include <mrpt/math/types_math.h>  // Eigen must be included first via MRPT to enable the plugin system
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <mrpt/otherlibs/do_opencv_includes.h>

namespace mrpt::vision::pnp
{
/** \addtogroup pnp Perspective-n-Point pose estimation
 *  \ingroup mrpt_vision_grp
 *  @{
 */

/**
 * @class p3p
 * @author Chandra Mangipudi
 * @date 11/08/16
 * @file p3p.h
 * @brief P3P Pose estimation Algorithm - Eigen Implementation
 */
class p3p
{
   public:
	//! Constructor for p3p class using C
	p3p(double fx, double fy, double cx, double cy);

	//! Constructor using Eigen matrix
	p3p(Eigen::MatrixXd cam_intrinsic);

#if MRPT_HAS_OPENCV
	//! Constructor using OpenCV matrix
	p3p(cv::Mat cameraMatrix);

	/**
	 * @brief Function to compute pose by P3P using OpenCV
	 * @param[out] R Rotation Matrix
	 * @param[out] tvec Translation Vector
	 * @param[in] opoints Object points
	 * @param[in] ipoints Image points
	 * @return Success flag
	 */
	bool solve(
		cv::Mat& R, cv::Mat& tvec, const cv::Mat& opoints,
		const cv::Mat& ipoints);
#endif
	bool solve(
		Eigen::Ref<Eigen::Matrix3d> R, Eigen::Ref<Eigen::Vector3d> t,
		Eigen::MatrixXd obj_pts, Eigen::MatrixXd img_pts);

	/**
	 * @brief Function to compute pose from 3 points using C function
	 * @param[out] R Rotation Matrix
	 * @param[out] t Translation Vector
	 * @param[in] mu0 x- coordinate of image point 1
	 * @param[in] mv0 y- coordinate of image point 1
	 * @param[in] X0  X- coordinate of object point 1
	 * @param[in] Y0  Y- coordinate of object point 1
	 * @param[in] Z0  Z- coordinate of object point 1
	 * @param[in] mu1 x- coordinate of image point 2
	 * @param[in] mv1 y- coordinate of image point 2
	 * @param[in] X1  X- coordinate of object point 2
	 * @param[in] Y1  Y- coordinate of object point 2
	 * @param[in] Z1  Z- coordinate of object point 2
	 * @param[in] mu2 x- coordinate of image point 3
	 * @param[in] mv2 y- coordinate of image point 3
	 * @param[in] X2  X- coordinate of object point 3
	 * @param[in] Y2  Y- coordinate of object point 3
	 * @param[in] Z2  Z- coordinate of object point 3
	 * @return Success flag
	 */
	int solve(
		double R[4][3][3], double t[4][3], double mu0, double mv0, double X0,
		double Y0, double Z0, double mu1, double mv1, double X1, double Y1,
		double Z1, double mu2, double mv2, double X2, double Y2, double Z2);

	/**
	 * @brief Function to compute pose from 4 points using C function
	 * @param[out] R Rotation Matrix
	 * @param[out] t Translation Vector
	 * @param[in] mu0 x- coordinate of image point 1
	 * @param[in] mv0 y- coordinate of image point 1
	 * @param[in] X0  X- coordinate of object point 1
	 * @param[in] Y0  Y- coordinate of object point 1
	 * @param[in] Z0  Z- coordinate of object point 1
	 * @param[in] mu1 x- coordinate of image point 2
	 * @param[in] mv1 y- coordinate of image point 2
	 * @param[in] X1  X- coordinate of object point 2
	 * @param[in] Y1  Y- coordinate of object point 2
	 * @param[in] Z1  Z- coordinate of object point 2
	 * @param[in] mu2 x- coordinate of image point 3
	 * @param[in] mv2 y- coordinate of image point 3
	 * @param[in] X2  X- coordinate of object point 3
	 * @param[in] Y2  Y- coordinate of object point 3
	 * @param[in] Z2  Z- coordinate of object point 3
	 * @param[in] mu3 x- coordinate of image point 4
	 * @param[in] mv3 y- coordinate of image point 4
	 * @param[in] X3  X- coordinate of object point 4
	 * @param[in] Y3  Y- coordinate of object point 4
	 * @param[in] Z3  Z- coordinate of object point 4
	 * @return
	 */
	bool solve(
		double R[3][3], double t[3], double mu0, double mv0, double X0,
		double Y0, double Z0, double mu1, double mv1, double X1, double Y1,
		double Z1, double mu2, double mv2, double X2, double Y2, double Z2,
		double mu3, double mv3, double X3, double Y3, double Z3);

   private:
#if MRPT_HAS_OPENCV
	/**
	 * @brief OpenCV Initialization function to access camera intrinsic matrix
	 * @param[in] cameraMatrix Camera Intrinsic Matrix in OpenCV Mat format
	 */
	template <typename T>
	void init_camera_parameters(const cv::Mat& cameraMatrix)
	{
		cx = cameraMatrix.at<T>(0, 2);
		cy = cameraMatrix.at<T>(1, 2);
		fx = cameraMatrix.at<T>(0, 0);
		fy = cameraMatrix.at<T>(1, 1);
	}

	/**
	 * @brief OpoenCV wrapper for extracting object and image points
	 * @param[in] opoints Object points (OpenCV Mat)
	 * @param[in] ipoints Image points (OpenCV Mat)
	 * @param[out] points Combination of object and image points (C structure)
	 */
	template <typename OpointType, typename IpointType>
	void extract_points(
		const cv::Mat& opoints, const cv::Mat& ipoints,
		std::vector<double>& points)
	{
		points.clear();
		points.resize(20);
		for (int i = 0; i < 4; i++)
		{
			points[i * 5] = ipoints.at<IpointType>(i).x * fx + cx;
			points[i * 5 + 1] = ipoints.at<IpointType>(i).y * fy + cy;
			points[i * 5 + 2] = opoints.at<OpointType>(i).x;
			points[i * 5 + 3] = opoints.at<OpointType>(i).y;
			points[i * 5 + 4] = opoints.at<OpointType>(i).z;
		}
	}
#endif

	/**
	 * @brief Eigen wrapper for extracting object and image points
	 * @param[in] opoints Object points (Eigen Mat)
	 * @param[in] ipoints Image points (Eigen Mat)
	 * @param[out] points Combination of object and image points (C structure)
	 */
	void extract_points(
		Eigen::MatrixXd obj_pts, Eigen::MatrixXd img_pts,
		std::vector<double>& points)
	{
		points.clear();
		points.resize(20);
		for (int i = 0; i < 4; i++)
		{
			points[i * 5] = img_pts(i, 0) * fx + cx;
			points[i * 5 + 1] = img_pts(i, 1) * fy + cy;
			points[i * 5 + 2] = obj_pts(i, 0);
			points[i * 5 + 3] = obj_pts(i, 1);
			points[i * 5 + 4] = obj_pts(i, 2);
		}
	}
	/**
	 * @brief Function to compute inverse parameters of camera intrinsic matrix
	 */
	void init_inverse_parameters();

	/**
	 * @brief Helper function to @func solve()
	 * @param[out] lengths Internal lengths used for P3P
	 * @param[in] distances Internal distances used for computation of lengths
	 * @param[in] cosines Internal cosines used for computation of lengths
	 * @return
	 */
	int solve_for_lengths(
		double lengths[4][3], double distances[3], double cosines[3]);
	bool align(
		double M_start[3][3], double X0, double Y0, double Z0, double X1,
		double Y1, double Z1, double X2, double Y2, double Z2, double R[3][3],
		double T[3]);
	/**
	 * @brief Function used to compute the SVD
	 * @param[in] A Input Matrix for which SVD is to be computed
	 * @param[out] D Diagonal Matrix
	 * @param[out] U Matrix of left eigen vectors
	 * @return
	 */
	bool jacobi_4x4(double* A, double* D, double* U);

	double fx;  //! Focal length x
	double fy;  //! Focal length y
	double cx;  //! Image center x
	double cy;  //! Image center y
	double inv_fx;  //! Inverse of focal length x
	double inv_fy;  //! Inverse of focal length y
	double cx_fx;  //! Inverse of image center point x
	double cy_fy;  //! Inverse of image center point y
};

/** @}  */  // end of grouping
}  // namespace mrpt::vision::pnp
