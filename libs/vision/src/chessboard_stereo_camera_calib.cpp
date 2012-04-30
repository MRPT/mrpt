/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2012  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

#include <mrpt/vision.h>  // Precompiled headers

#include <mrpt/system/filesystem.h>
#include <mrpt/utils/CConfigFileMemory.h>
#include <mrpt/math/geometry.h>

#include <mrpt/vision/chessboard_stereo_camera_calib.h>
#include <mrpt/vision/pinhole.h>
#include <mrpt/vision/chessboard_camera_calib.h>

#include "do_opencv_includes.h"

using namespace mrpt;
using namespace mrpt::vision;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace std;

// State of the Lev-Marq optimization:
struct lm_stat_t
{
	const TCalibrationStereoImageList & images;
	const vector<size_t> & valid_image_pair_indices;
	const vector<TPoint3D> &obj_points;

	// State being optimized:
	//  N*left_cam_pose + right2left_pose + left_cam_params + right_cam_params
	mrpt::aligned_containers<CPose3D>::vector_t left_cam_poses;  // Poses of the origin of coordinates of the pattern wrt the left camera
	CPose3D         right2left_pose;
	CArrayDouble<9> left_cam_params, right_cam_params; // [fx fy cx cy k1 k2 k3 t1 t2]


	// Ctor
	lm_stat_t(
	const TCalibrationStereoImageList & _images,
	const vector<size_t> & _valid_image_pair_indices,
	const vector<TPoint3D> &_obj_points
	) : images(_images), valid_image_pair_indices(_valid_image_pair_indices), obj_points(_obj_points)
	{
		left_cam_poses.assign(images.size(), CPose3D(0,0,1, 0,0,0) );  // Initial
	}

	// Swap:
	void swap(lm_stat_t &o)
	{
		left_cam_poses.swap( o.left_cam_poses );
		std::swap( right2left_pose, o.right2left_pose );
		std::swap( left_cam_params, o.left_cam_params );
		std::swap( right_cam_params, o.right_cam_params );
	}
};

/** Data associated to *each observation* in the Lev-Marq. model */
struct TResidJacobElement
{
	Eigen::Matrix<double,4,1>  predicted_obs;  //!< [u_l v_l  u_r v_r]: left/right camera pixels
	Eigen::Matrix<double,4,1>  residual;       //!<  = predicted_obs - observations
	Eigen::Matrix<double,4,30> J; //!< Jacobian. 4=the two predicted pixels; 30=Read below for the meaning of these 30 variables
};

typedef vector< mrpt::aligned_containers<TResidJacobElement>::vector_t > TResidualJacobianList;

// Auxiliary functions for the Lev-Marq algorithm:
double recompute_errors_and_Jacobians(const lm_stat_t & lm_stat, TResidualJacobianList &res_jac);
void build_linear_system(const TResidualJacobianList  & res_jac,  const vector_size_t & var_indxs, Eigen::VectorXd &minus_g, Eigen::MatrixXd &H);
void add_lm_increment(const Eigen::VectorXd &eps, const vector_size_t & var_indxs, lm_stat_t &new_lm_stat);

/* -------------------------------------------------------
				checkerBoardStereoCalibration
   ------------------------------------------------------- */
bool mrpt::vision::checkerBoardStereoCalibration(
	TCalibrationStereoImageList & images,
	const TStereoCalibParams    & p,
	TStereoCalibResults         & out
	)
{
#if MRPT_HAS_OPENCV
	try
	{
		ASSERT_(p.check_size_x>2)
		ASSERT_(p.check_size_y>2)
		ASSERT_(p.check_squares_length_X_meters>0)
		ASSERT_(p.check_squares_length_Y_meters>0)

		if (images.size()<1)
		{
			std::cout << "ERROR: No input images." << std::endl;
			return false;
		}

		const unsigned   CORNERS_COUNT = p.check_size_x * p.check_size_y;
		const TImageSize check_size    = TImageSize(p.check_size_x, p.check_size_y);

		// For each image, find checkerboard corners:
		// -----------------------------------------------
        unsigned int valid_detected_stereo_imgs = 0;

		// Left/Right sizes (may be different)
        TImageSize imgSize[2] = { TImageSize(0,0), TImageSize(0,0) };

		//vector<string>   pointsIdx2imageFile;
		vector<size_t>   valid_image_pair_indices; // Indices in images[] which are valid pairs to be used in the optimization

		int find_chess_flags = CV_CALIB_CB_ADAPTIVE_THRESH;
		if (p.normalize_image) find_chess_flags |= CV_CALIB_CB_NORMALIZE_IMAGE;

		for (size_t i=0;i<images.size();i++)
		{
			// Do loop for each left/right image:
			TImageCalibData *dats[2] = { &images[i].left, &images[i].right };
			bool corners_found[2]={ false, false };

			for (int lr=0;lr<2;lr++)
			{
				TImageCalibData	&dat = *dats[lr];
				dat.detected_corners.clear();

				// Make grayscale version:
				const CImage img_gray( dat.img_original, FAST_REF_OR_CONVERT_TO_GRAY );

				if (!i)
				{
					imgSize[lr] = img_gray.getSize();
					if (lr==0) {
						out.cam_params.leftCamera.ncols = imgSize[lr].x;
						out.cam_params.leftCamera.nrows = imgSize[lr].y;
					}
					else {
						out.cam_params.rightCamera.ncols = imgSize[lr].x;
						out.cam_params.rightCamera.nrows = imgSize[lr].y;
					}
				}
				else
				{
					if (imgSize[lr].y != (int)img_gray.getHeight() || imgSize[lr].x != (int)img_gray.getWidth())
					{
						std::cout << "ERROR: All the images in each left/right channel must have the same size." << std::endl;
						return false;
					}
				}

				// Do detection (this includes the "refine corners" with cvFindCornerSubPix):
				corners_found[lr] = mrpt::vision::findChessboardCorners(
					img_gray,
					dat.detected_corners,
					p.check_size_x,p.check_size_y,
					p.normalize_image // normalize_image
					);

				if (corners_found[lr] && dat.detected_corners.size()!=CORNERS_COUNT)
					corners_found[lr] = false;

				if (p.verbose) cout << format("%s img #%u: %s\n", lr==0 ? "LEFT":"RIGHT", static_cast<unsigned int>(i), corners_found[lr] ? "DETECTED" : "NOT DETECTED" );

				if( corners_found[lr] )
				{
					// Draw the checkerboard in the corresponding image:
					if ( !dat.img_original.isExternallyStored() )
					{
						// Checkboad as color image:
						dat.img_original.colorImage( dat.img_checkboard );
						dat.img_checkboard.drawChessboardCorners( dat.detected_corners,check_size.x,check_size.y);
					}
				}

			} // end for lr

			// We just finished detecting corners in a left/right pair.
			// Only if corners were detected perfectly in BOTH images, add this image pair as a good one for optimization:
			if( corners_found[0] && corners_found[1] )
				valid_image_pair_indices.push_back(i);

		} // end find corners

		if (p.verbose) std::cout << valid_image_pair_indices.size() << " valid image pairs." << std::endl;
		if (valid_image_pair_indices.empty() )
		{
			std::cout << "ERROR: No valid images. Perhaps the checkerboard size is incorrect?" << std::endl;
			return false;
		}

		//  Create reference coordinates of the detected corners, in "global" coordinates:
		// -------------------------------------------------------------------------------
		vector<TPoint3D> obj_points;
		obj_points.reserve(CORNERS_COUNT);
		for( unsigned int y = 0; y < p.check_size_y; y++ )
			for( unsigned int x = 0; x < p.check_size_x; x++  )
				obj_points.push_back( TPoint3D(
					p.check_squares_length_X_meters * x,
					p.check_squares_length_Y_meters * y,
					0 ));


		// ----------------------------------------------------------------------------------
		//  Levenberg-Marquardt algorithm
		//
		//  State space (poses are actually on-manifold increments):
		//     N*left_cam_pose + right2left_pose + left_cam_params + right_cam_params
		//       N * 6         +        6        +        9        +         9       = 6N+24
		// ----------------------------------------------------------------------------------
		const size_t N = valid_image_pair_indices.size();
		const size_t nObs = 2 * N * CORNERS_COUNT;  // total number of valid observations (px,py) taken into account in the optimization

		const double tau = 0.3;
		const double t1 = 1e-8;
		const double t2 = 1e-8;
		const size_t maxIters = 100;

		// Select which cam. parameters to optimize (for each camera!) and which to leave fixed:
		// Indices:  [0   1  2  3  4  5  6  7  8]
		// Variables:[fx fy cx cy k1 k2 k3 t1 t2]
		vector_size_t  vars_to_optimize; 
		vars_to_optimize.push_back(0);
		vars_to_optimize.push_back(1);
		vars_to_optimize.push_back(2);
		vars_to_optimize.push_back(3);
		//... vars_to_optimize.push_back(4);

		//  * N x Manifold Epsilon of left camera pose (6)
		//  * Manifold Epsilon of right2left pose (6)
		//  * Left-cam-params (9)
		//  * Right-cam-params (9)
		const size_t nUnknowns = N*6 + 6+ 2*vars_to_optimize.size();

		// Initial state:
		//Within lm_stat: CArrayDouble<9> left_cam_params, right_cam_params; // [fx fy cx cy k1 k2 k3 t1 t2]
		lm_stat_t        lm_stat(images,valid_image_pair_indices,obj_points);
	
		lm_stat.left_cam_params[0] = imgSize[0].x * 0.9;
		lm_stat.left_cam_params[1] = imgSize[0].y * 0.9;
		lm_stat.left_cam_params[2] = imgSize[0].x * 0.5;
		lm_stat.left_cam_params[3] = imgSize[0].y * 0.5;
		lm_stat.left_cam_params[4] = lm_stat.left_cam_params[5] = lm_stat.left_cam_params[6] = 
		lm_stat.left_cam_params[7] = lm_stat.left_cam_params[8] = 0;

		lm_stat.right_cam_params[0] = imgSize[1].x * 0.9;
		lm_stat.right_cam_params[1] = imgSize[1].y * 0.9;
		lm_stat.right_cam_params[2] = imgSize[1].x * 0.5;
		lm_stat.right_cam_params[3] = imgSize[1].y * 0.5;
		lm_stat.right_cam_params[4] = lm_stat.right_cam_params[5] = lm_stat.right_cam_params[6] = 
		lm_stat.right_cam_params[7] = lm_stat.right_cam_params[8] = 0;


		// Residuals & Jacobians:
		TResidualJacobianList res_jacob;
		double err = recompute_errors_and_Jacobians(lm_stat,res_jacob);

		if (p.verbose) cout << "LM iter#0: Initial sqr.err=" << err << " avr.err(px):"<< std::sqrt(err/nObs)<< endl;

		// Build linear system:
		Eigen::VectorXd  minus_g; // minus gradient
		Eigen::MatrixXd  H;       // Hessian matrix
		build_linear_system(res_jacob,vars_to_optimize,minus_g,H);

		ASSERT_EQUAL_(nUnknowns, H.cols())

		// Lev-Marq. parameters:
		double nu = 2;
		double lambda = tau * H.diagonal().array().maxCoeff();
		bool   done = (minus_g.array().abs().maxCoeff() < t1);

		// Lev-Marq. main loop:
		size_t iter=0;
		while (iter<maxIters && !done)
		{
			// Solve for increment: (H + \lambda I) eps = -gradient
			Eigen::MatrixXd  HH = H;
			for (size_t i=0;i<nUnknowns;i++) HH(i,i)+=lambda;

			Eigen::LLT<Eigen::MatrixXd> llt(HH);
			if (llt.info()!=Eigen::Success)
			{
				lambda *= nu;
				nu *= 2;
				done = (lambda>1e10);
				
				if (p.verbose && !done) cout << "LM iter#"<<iter<<": Couldn't solve LLt, retrying with large 'lambda'=" << lambda << endl;
				continue;
			}
			const Eigen::VectorXd eps = llt.solve(minus_g);

			const double eps_norm = eps.norm();
			if (eps_norm < t2*(eps_norm+t2))
			{
				done=true;
				break;
			}

			// Tentative new state:
			lm_stat_t new_lm_stat(lm_stat); //images,valid_image_pair_indices,obj_points);
			add_lm_increment(eps,vars_to_optimize, new_lm_stat);

			TResidualJacobianList new_res_jacob;

			// discriminant:
			double err_new = recompute_errors_and_Jacobians(new_lm_stat, new_res_jacob);

			const double l = (err-err_new)/ (eps.array()*(lambda*eps + minus_g).array() ).sum();
			if(l>0)
			{
				// Good: Accept new values
				if (p.verbose) cout << "LM iter#"<<iter<<": New total sqr.err=" << err_new << " avr.err(px):"<< std::sqrt(err/nObs)  <<"->" <<  std::sqrt(err_new/nObs) << endl;

				// swap: faster than "lm_stat <- new_lm_stat"
				lm_stat.swap( new_lm_stat );
				res_jacob.swap( new_res_jacob );

				err = err_new;
				build_linear_system(res_jacob,vars_to_optimize,minus_g,H);

				// Too small gradient?
				done = (minus_g.array().abs().maxCoeff() < t1);
				lambda *= max(1.0/3.0, 1-std::pow(2*l-1,3.0) );
				nu = 2.0;

				iter++;
			}
			else
			{
				if (p.verbose) cout << "LM iter#"<<iter<<": No update: err=" << err << " -> err_new=" << err_new << endl;
				lambda *= nu;
				nu *= 2.0;
				done = (lambda>1e10);
			}

		} // end while
		// -------------------------------------------------------------------------------
		//  End of Levenberg-Marquardt
		// -------------------------------------------------------------------------------

		return true;
	}
	catch(std::exception &e)
	{
		std::cout << e.what() << std::endl;
		return false;
	}
#else
	THROW_EXCEPTION("Function not available: MRPT was compiled without OpenCV")
#endif
}

/*
Jacobian:

 d b( [px py pz] )
------------------  (5x3) =
  d{ px py pz }

[                                  1/pz,                                     0,                                                     -px/pz^2]
[                                     0,                                  1/pz,                                                     -py/pz^2]
[                           (2*px)/pz^2,                           (2*py)/pz^2,                              - (2*px^2)/pz^3 - (2*py^2)/pz^3]
[   (4*px*(px^2/pz^2 + py^2/pz^2))/pz^2,   (4*py*(px^2/pz^2 + py^2/pz^2))/pz^2,   -2*(px^2/pz^2 + py^2/pz^2)*((2*px^2)/pz^3 + (2*py^2)/pz^3)]
[ (6*px*(px^2/pz^2 + py^2/pz^2)^2)/pz^2, (6*py*(px^2/pz^2 + py^2/pz^2)^2)/pz^2, -3*(px^2/pz^2 + py^2/pz^2)^2*((2*px^2)/pz^3 + (2*py^2)/pz^3)]

\left(\begin{array}{ccc} \frac{1}{\mathrm{pz}} & 0 & -\frac{\mathrm{px}}{{\mathrm{pz}}^2}\\ 0 & \frac{1}{\mathrm{pz}} & -\frac{\mathrm{py}}{{\mathrm{pz}}^2}\\ \frac{2\, \mathrm{px}}{{\mathrm{pz}}^2} & \frac{2\, \mathrm{py}}{{\mathrm{pz}}^2} &  - \frac{2\, {\mathrm{px}}^2}{{\mathrm{pz}}^3} - \frac{2\, {\mathrm{py}}^2}{{\mathrm{pz}}^3}\\ \frac{4\, \mathrm{px}\, \left(\frac{{\mathrm{px}}^2}{{\mathrm{pz}}^2} + \frac{{\mathrm{py}}^2}{{\mathrm{pz}}^2}\right)}{{\mathrm{pz}}^2} & \frac{4\, \mathrm{py}\, \left(\frac{{\mathrm{px}}^2}{{\mathrm{pz}}^2} + \frac{{\mathrm{py}}^2}{{\mathrm{pz}}^2}\right)}{{\mathrm{pz}}^2} & - 2\, \left(\frac{{\mathrm{px}}^2}{{\mathrm{pz}}^2} + \frac{{\mathrm{py}}^2}{{\mathrm{pz}}^2}\right)\, \left(\frac{2\, {\mathrm{px}}^2}{{\mathrm{pz}}^3} + \frac{2\, {\mathrm{py}}^2}{{\mathrm{pz}}^3}\right)\\ \frac{6\, \mathrm{px}\, {\left(\frac{{\mathrm{px}}^2}{{\mathrm{pz}}^2} + \frac{{\mathrm{py}}^2}{{\mathrm{pz}}^2}\right)}^2}{{\mathrm{pz}}^2} & \frac{6\, \mathrm{py}\, {\left(\frac{{\mathrm{px}}^2}{{\mathrm{pz}}^2} + \frac{{\mathrm{py}}^2}{{\mathrm{pz}}^2}\right)}^2}{{\mathrm{pz}}^2} & - 3\, {\left(\frac{{\mathrm{px}}^2}{{\mathrm{pz}}^2} + \frac{{\mathrm{py}}^2}{{\mathrm{pz}}^2}\right)}^2\, \left(\frac{2\, {\mathrm{px}}^2}{{\mathrm{pz}}^3} + \frac{2\, {\mathrm{py}}^2}{{\mathrm{pz}}^3}\right) \end{array}\right)

*/ 

void jacob_db_dp(
	const TPoint3D &p,   // 3D coordinates wrt the camera
	Eigen::Matrix<double,5,3> &G)
{
	const double pz_ = 1/p.z;
	const double pz_2 = pz_*pz_;
	const double pz_3 = pz_2*pz_;

	G(0,0) = pz_; 
	G(0,1) = 0;
	G(0,2) = -p.x * pz_2;

	G(1,0) = 0;
	G(1,1) = pz_;
	G(1,2) = -p.x * pz_2;

	G(2,0) = 2*p.x * pz_2;
	G(2,1) = 2*p.y * pz_2;
	G(2,2) = -2*p.x*p.x*pz_3 - 2*p.y*p.y*pz_3;

	G(3,0) = (4*p.x*(p.x*p.x*pz_2 + p.y*p.y*pz_2))*pz_2;
	G(3,1) = (4*p.y*(p.x*p.x*pz_2 + p.y*p.y*pz_2))*pz_2;
	G(3,2) = -2*(p.x*p.x *pz_2 + p.y*p.y*pz_2)*((2*p.x*p.x*pz_3) + (2*p.y*p.y)*pz_3);

	G(4,0) = (6*p.x* square(p.x*p.x*pz_2 + p.y*p.y*pz_2) )*pz_2;
	G(4,1) = (6*p.y* square(p.x*p.x*pz_2 + p.y*p.y*pz_2) )*pz_2;
	G(4,2) = -3*square(p.x*p.x *pz_2 + p.y*p.y*pz_2) * ((2*p.x*p.x)*pz_3 + (2*p.y*p.y)*pz_3);
}

/*
Jacobian:

 dh( b,c )
----------- = Hb  | 2x5
  d{ b }

[ fx*(k1*r2 + k2*r4 + k3*r6 + 4*t2*x + 2*t1*y + 1),                                        2*fx*t1*x, fx*(t2 + k1*x), fx*k2*x, fx*k3*x]
[                                        2*fy*t2*y, fy*(k1*r2 + k2*r4 + k3*r6 + 2*t2*x + 4*t1*y + 1), fy*(t1 + k1*y), fy*k2*y, fy*k3*y]

\left(\begin{array}{ccccc} \mathrm{fx}\, \left(\mathrm{k1}\, \mathrm{r2} + \mathrm{k2}\, \mathrm{r4} + \mathrm{k3}\, \mathrm{r6} + 4\, \mathrm{t2}\, x + 2\, \mathrm{t1}\, y + 1\right) & 2\, \mathrm{fx}\, \mathrm{t1}\, x & \mathrm{fx}\, \left(\mathrm{t2} + \mathrm{k1}\, x\right) & \mathrm{fx}\, \mathrm{k2}\, x & \mathrm{fx}\, \mathrm{k3}\, x\\ 2\, \mathrm{fy}\, \mathrm{t2}\, y & \mathrm{fy}\, \left(\mathrm{k1}\, \mathrm{r2} + \mathrm{k2}\, \mathrm{r4} + \mathrm{k3}\, \mathrm{r6} + 2\, \mathrm{t2}\, x + 4\, \mathrm{t1}\, y + 1\right) & \mathrm{fy}\, \left(\mathrm{t1} + \mathrm{k1}\, y\right) & \mathrm{fy}\, \mathrm{k2}\, y & \mathrm{fy}\, \mathrm{k3}\, y \end{array}\right)

 dh( b,c )
----------- = Hc  | 2x9
  d{ c }

[ t2*(2*x^2 + r2) + x*(k1*r2 + k2*r4 + k3*r6 + 1) + 2*t1*x*y,                                                          0, 1, 0, fx*r2*x, fx*r4*x, fx*r6*x,        2*fx*x*y, fx*(2*x^2 + r2)]
[                                                          0, t1*(2*y^2 + r2) + y*(k1*r2 + k2*r4 + k3*r6 + 1) + 2*t2*x*y, 0, 1, fy*r2*y, fy*r4*y, fy*r6*y, fy*(2*y^2 + r2),        2*fy*x*y]

\left(\begin{array}{ccccccccc} \mathrm{t2}\, \left(2\, x^2 + \mathrm{r2}\right) + x\, \left(\mathrm{k1}\, \mathrm{r2} + \mathrm{k2}\, \mathrm{r4} + \mathrm{k3}\, \mathrm{r6} + 1\right) + 2\, \mathrm{t1}\, x\, y & 0 & 1 & 0 & \mathrm{fx}\, \mathrm{r2}\, x & \mathrm{fx}\, \mathrm{r4}\, x & \mathrm{fx}\, \mathrm{r6}\, x & 2\, \mathrm{fx}\, x\, y & \mathrm{fx}\, \left(2\, x^2 + \mathrm{r2}\right)\\ 0 & \mathrm{t1}\, \left(2\, y^2 + \mathrm{r2}\right) + y\, \left(\mathrm{k1}\, \mathrm{r2} + \mathrm{k2}\, \mathrm{r4} + \mathrm{k3}\, \mathrm{r6} + 1\right) + 2\, \mathrm{t2}\, x\, y & 0 & 1 & \mathrm{fy}\, \mathrm{r2}\, y & \mathrm{fy}\, \mathrm{r4}\, y & \mathrm{fy}\, \mathrm{r6}\, y & \mathrm{fy}\, \left(2\, y^2 + \mathrm{r2}\right) & 2\, \mathrm{fy}\, x\, y \end{array}\right)

*/

void jacob_dh_db_and_dh_dc(
	const TPoint3D & nP,  // Point in relative coords wrt the camera
	const Eigen::Matrix<double,9,1>  & c,  // camera parameters
	Eigen::Matrix<double,2,5>  & Hb,
	Eigen::Matrix<double,2,9> & Hc
	)
{
	const double x = nP.x/nP.z;
	const double y = nP.y/nP.z;

	const double r2 = x*x + y*y;
	const double r4 = r2*r2;
	const double r6 = r4*r2;

	const double fx=c[0], fy=c[1]; 
	const double cx=c[2], cy=c[3];
	const double k1=c[4], k2=c[5], k3=c[6];
	const double t1=c[7], t2=c[8];

	// Hb = dh(b,c)/db  (2x5)
	Hb(0,0) = fx*(k1*r2 + k2*r4 + k3*r6 + 4*t2*x + 2*t1*y + 1);
	Hb(0,1) = 2*fx*t1*x;
	Hb(0,2) = fx*(t2 + k1*x);
	Hb(0,3) = fx*k2*x;
	Hb(0,4) = fx*k3*x;
	
	Hb(1,0) = 2*fy*t2*y;
	Hb(1,1) = fy*(k1*r2 + k2*r4 + k3*r6 + 2*t2*x + 4*t1*y + 1);
	Hb(1,2) = fy*(t1 + k1*y);
	Hb(1,3) = fy*k2*y;
	Hb(1,4) = fy*k3*y;

	// Hc = dh(b,c)/dc  (2x9)
	Hc(0,0) = t2*(2*x*x + r2) + x*(k1*r2 + k2*r4 + k3*r6 + 1) + 2*t1*x*y;
	Hc(0,1) = 0;
	Hc(0,2) = 1;
	Hc(0,3) = 0;
	Hc(0,4) = fx*r2*x;
	Hc(0,5) = fx*r4*x;
	Hc(0,6) = fx*r6*x;
	Hc(0,7) = 2*fx*x*y;
	Hc(0,8) = fx*(2*x*x + r2);

	Hc(1,0) = 0;
	Hc(1,1) = t1*(2*y*y + r2) + y*(k1*r2 + k2*r4 + k3*r6 + 1) + 2*t2*x*y;
	Hc(1,2) = 0;
	Hc(1,3) = 1;
	Hc(1,4) = fy*r2*y;
	Hc(1,5) = fy*r4*y;
	Hc(1,6) = fy*r6*y;
	Hc(1,7) = fy*(2*y*y + r2);
	Hc(1,8) = 2*fy*x*y;
}

void jacob_deps_D_p_deps(
	const TPoint3D &p_D,   // D (+) p
	Eigen::Matrix<double,3,6> &dpl_del)
{
	// Jacobian 10.3.4 in technical report "A tutorial on SE(3) transformation parameterizations and on-manifold optimization"
	dpl_del.block<3,3>(0,0).setIdentity();
	dpl_del.block<3,3>(0,3) = mrpt::math::skew_symmetric3_neg(p_D);
}

void jacob_dA_eps_D_p_deps(
	const CPose3D  &A, 
	const CPose3D  &D, 
	const TPoint3D &p,
	Eigen::Matrix<double,3,6> &dp_deps)
{
	// Jacobian 10.3.7 in technical report "A tutorial on SE(3) transformation parameterizations and on-manifold optimization"
	const Eigen::Matrix<double,1,3> P(p.x,p.y,p.z); 
	const Eigen::Matrix<double,1,3> dr1 = D.getRotationMatrix().block<1,3>(0,0);
	const Eigen::Matrix<double,1,3> dr2 = D.getRotationMatrix().block<1,3>(1,0);
	const Eigen::Matrix<double,1,3> dr3 = D.getRotationMatrix().block<1,3>(2,0);

	const Eigen::Matrix<double,1,3> v( P.dot(dr1)+D.x(), P.dot(dr2)+D.y(), P.dot(dr3)+D.z() );

	Eigen::Matrix<double,3,6> H;
	H.block<3,3>(0,0).setIdentity();
	H.block<3,3>(0,3) = mrpt::math::skew_symmetric3(v);

	dp_deps.noalias() = A.getRotationMatrix() * H;
}


void project_point(
	const mrpt::math::TPoint3D & P,
	const mrpt::utils::TCamera & params,
	const CPose3D              & cameraPose,
	mrpt::vision::TPixelCoordf & px
	)
{
	// Change the reference system to that wrt the camera
	TPoint3D nP;
	cameraPose.inverseComposePoint( P.x, P.y, P.z, nP.x, nP.y, nP.z );

	// Pinhole model:
	const double x = nP.x/nP.z;
	const double y = nP.y/nP.z;

	// Radial distortion:
	const double r2 = square(x)+square(y);
	const double r4 = square(r2);
	const double r6 = r2*r4;
	const double A  = 1+params.dist[0]*r2+params.dist[1]*r4+params.dist[4]*r6;
	const double B  = 2*x*y;

	px.x = params.cx() + params.fx() * ( x*A + params.dist[2]*B + params.dist[3]*(r2+2*square(x)) );
	px.y = params.cy() + params.fy() * ( y*A + params.dist[3]*B + params.dist[2]*(r2+2*square(y)) );
}

// Build the "-gradient" and the Hessian matrix:
// Variables are in these order:
//  * N x Manifold Epsilon of left camera pose (6)
//  * Manifold Epsilon of right2left pose (6)
//  * Left-cam-params (9)
//  * Right-cam-params (9)
void build_linear_system(
	const TResidualJacobianList  & res_jac,  
	const vector_size_t & var_indxs,
	Eigen::VectorXd &minus_g,
	Eigen::MatrixXd &H)
{
	const size_t N = res_jac.size();  // Number of stereo image pairs
	const size_t nMaxUnknowns = N*6+6+9+9;

	// Reset to zeros:
	Eigen::VectorXd minus_g_tot = Eigen::VectorXd::Zero(nMaxUnknowns);
	Eigen::MatrixXd H_tot       = Eigen::MatrixXd::Zero(nMaxUnknowns,nMaxUnknowns);

	// Sum the contribution from each observation:
	for (size_t i=0;i<N;i++)
	{
		const size_t nObs = res_jac[i].size();
		for (size_t j=0;j<nObs;j++)
		{
			const TResidJacobElement & rje = res_jac[i][j];
			// Sub-Hessian & sub-gradient are variables in this order: 
			//  eps_left_cam, eps_right2left_cam, left_cam_params, right_cam_params
			const Eigen::Matrix<double,30,30> Hij = rje.J.transpose() * rje.J;
			const Eigen::Matrix<double,30,1>  gij = rje.J.transpose() * rje.residual;
			
			// Assemble in their place:
			minus_g_tot.block<6,1>(i*6,0)     -= gij.block<6,1>(0,0);
			minus_g_tot.block<6+9+9,1>(N*6,0) -= gij.block<6+9+9,1>(6,0);

			H_tot.block<6,6>(i*6,i*6) += Hij.block<6,6>(0,0);

			H_tot.block<6+9+9,6+9+9>(N*6,N*6) += Hij.block<6+9+9,6+9+9>(6,6);
			H_tot.block<6,6+9+9>(i*6,N*6) += Hij.block<6,6+9+9>(0,6);
			H_tot.block<6+9+9,6>(N*6,i*6) += Hij.block<6+9+9,6>(6,0);
		}
	}

#if 0
	minus_g_tot.saveToTextFile("minus_g_tot.txt");
	H_tot.saveToTextFile("H_tot.txt");
#endif

	// Ignore those derivatives of "fixed" (non-optimizable) variables in camera parameters
	const size_t N_Cs = var_indxs.size();  // [0-8]
	const size_t nUnknowns = N*6 + 6 + 2*N_Cs;
	const size_t nUnkPoses = N*6 + 6;

	minus_g.setZero(nUnknowns);
	H.setZero(nUnknowns,nUnknowns);
	// Copy unmodified all but the cam. params:
	minus_g.block(0,0, nUnkPoses,1 )  = minus_g_tot.block(0,0, nUnkPoses,1);
	H.block(0,0, nUnkPoses,nUnkPoses) = H_tot.block(0,0, nUnkPoses,nUnkPoses);

	// Selective copy cam. params parts:
	for (size_t i=0;i<N_Cs;i++)
	{
		minus_g[nUnkPoses+i]      = minus_g_tot[nUnkPoses +     var_indxs[i] ];
		minus_g[nUnkPoses+N_Cs+i] = minus_g_tot[nUnkPoses + 9 + var_indxs[i] ];

		for (size_t j=0;j<N_Cs;j++)
		{
			H(nUnkPoses+i,nUnkPoses+j) = H_tot( nUnkPoses + var_indxs[i], nUnkPoses + var_indxs[j] );
			H(nUnkPoses+N_Cs+i,nUnkPoses+N_Cs+j) = H_tot( nUnkPoses +9+var_indxs[i], nUnkPoses +9+ var_indxs[j] );
		}
	}
} // end of build_linear_system


//  * N x Manifold Epsilon of left camera pose (6)
//  * Manifold Epsilon of right2left pose (6)
//  * Left-cam-params (9)
//  * Right-cam-params (9)
void add_lm_increment(
	const Eigen::VectorXd & eps, 
	const vector_size_t   & var_indxs, 
	lm_stat_t             & lm_stat)
{
	// Increment of the N cam poses
	const size_t N = lm_stat.valid_image_pair_indices.size();
	for (size_t i=0;i<N;i++)
	{
		const CPose3D &old_pose = lm_stat.left_cam_poses[ lm_stat.valid_image_pair_indices[i] ];

		// Use the Lie Algebra methods for the increment:
		const CArrayDouble<6> incr( &eps[6*i] );
		const CPose3D         incrPose = CPose3D::exp(incr);

		//new_pose =  old_pose  [+] delta
		//         = exp(delta) (+) old_pose
		CPose3D new_pose;
		new_pose.composeFrom(incrPose, old_pose);

		lm_stat.left_cam_poses[ lm_stat.valid_image_pair_indices[i] ] = new_pose;
	}

	// Increment of the right-left pose:
	{
		// Use the Lie Algebra methods for the increment:
		const CArrayDouble<6> incr( &eps[6*N] );
		const CPose3D         incrPose = CPose3D::exp(incr);

		lm_stat.right2left_pose = incrPose + lm_stat.right2left_pose;
	}

	// Increment of the camera params:
	const size_t idx=6*N+6;
	const size_t nPC = var_indxs.size();  // Number of camera parameters being optimized
	for (int i=0;i<nPC;i++)
	{
		lm_stat.left_cam_params [var_indxs[i]] += eps[idx+i];
		lm_stat.right_cam_params[var_indxs[i]] += eps[idx+nPC+i];
	}

} // end of add_lm_increment


// the 4x1 prediction are the (u,v) pixel coordinates for the left / right cameras:
double recompute_errors_and_Jacobians(
	const lm_stat_t       & lm_stat, 
	TResidualJacobianList & res_jac )
{
	double total_err=0;
	const size_t N = lm_stat.valid_image_pair_indices.size();
	res_jac.resize(N);

	//Parse lm_stat data: CArrayDouble<9> left_cam_params, right_cam_params; // [fx fy cx cy k1 k2 k3 t1 t2]
	TCamera camparam_l;
	camparam_l.fx( lm_stat.left_cam_params[0] ); camparam_l.fy( lm_stat.left_cam_params[1] );
	camparam_l.cx( lm_stat.left_cam_params[2] ); camparam_l.cy( lm_stat.left_cam_params[3] );
	camparam_l.k1( lm_stat.left_cam_params[4] ); camparam_l.k2( lm_stat.left_cam_params[5] ); camparam_l.k3( lm_stat.left_cam_params[6] );
	camparam_l.p1( lm_stat.left_cam_params[7] ); camparam_l.p2( lm_stat.left_cam_params[8] ); 
	TCamera camparam_r;
	camparam_r.fx( lm_stat.right_cam_params[0] ); camparam_r.fy( lm_stat.right_cam_params[1] );
	camparam_r.cx( lm_stat.right_cam_params[2] ); camparam_r.cy( lm_stat.right_cam_params[3] );
	camparam_r.k1( lm_stat.right_cam_params[4] ); camparam_r.k2( lm_stat.right_cam_params[5] ); camparam_r.k3( lm_stat.right_cam_params[6] );
	camparam_r.p1( lm_stat.right_cam_params[7] ); camparam_r.p2( lm_stat.right_cam_params[8] ); 

	// process all points from all N image pairs:
	for (size_t k=0;k<N;k++)
	{
		const size_t k_idx = lm_stat.valid_image_pair_indices[k];
		const size_t nPts = lm_stat.obj_points.size();
		res_jac[k].resize(nPts);
		// For each point in the pattern:
		for (size_t i=0;i<nPts;i++)
		{
			// Observed corners:
			const TPixelCoordf &px_obs_l = lm_stat.images[k_idx].left.detected_corners[i];
			const TPixelCoordf &px_obs_r = lm_stat.images[k_idx].right.detected_corners[i];
			const Eigen::Matrix<double,4,1> obs(px_obs_l.x,px_obs_l.y,px_obs_r.x,px_obs_r.y);

			TResidJacobElement & rje = res_jac[k][i];

			// Predict i'th corner on left & right cameras:
			// ---------------------------------------------
			TPixelCoordf px_l, px_r;
			project_point( lm_stat.obj_points[i], camparam_l, lm_stat.left_cam_poses[k], px_l );
			project_point( lm_stat.obj_points[i], camparam_r, lm_stat.right2left_pose+lm_stat.left_cam_poses[k], px_r );
			rje.predicted_obs = Eigen::Matrix<double,4,1>( px_l.x,px_l.y, px_r.x,px_r.y );

			// Residual:
			rje.residual = rje.predicted_obs - obs;
			
			// Accum. total squared error:
			total_err += rje.residual.squaredNorm();

			// ---------------------------------------------------------------------------------
			// Jacobian: (4x30)
			// See technical report cited in the headers for the theory behind these formulas.
			// ---------------------------------------------------------------------------------
			Eigen::Matrix<double,2,6> dhl_del, dhr_del, dhr_der;
			Eigen::Matrix<double,2,9> dhl_dcl, dhr_dcr;

			// 3D coordinates of the corner point wrt both cameras:
			TPoint3D pt_wrt_left, pt_wrt_right;
			lm_stat.left_cam_poses[k].composePoint(lm_stat.obj_points[i], pt_wrt_left );
			(lm_stat.right2left_pose+lm_stat.left_cam_poses[k]).composePoint(lm_stat.obj_points[i], pt_wrt_right);

			// Build partial Jacobians:
			Eigen::Matrix<double,2,5> dhl_dbl, dhr_dbr;
			jacob_dh_db_and_dh_dc(pt_wrt_left,  lm_stat.left_cam_params,  dhl_dbl, dhl_dcl );
			jacob_dh_db_and_dh_dc(pt_wrt_right, lm_stat.right_cam_params, dhr_dbr, dhr_dcr );

			Eigen::Matrix<double,5,3> dbl_dpl, dbr_dpr;
			jacob_db_dp(pt_wrt_left,  dbl_dpl);
			jacob_db_dp(pt_wrt_right, dbr_dpr);

			// p_l = exp(epsilon_l) (+) pose_left (+) point_ij
			// p_l = [exp(epsilon_r) (+) pose_right2left] (+) [exp(epsilon_l) (+) pose_left] (+) point_ij
			Eigen::Matrix<double,3,6> dpl_del, dpr_del, dpr_der;
			jacob_deps_D_p_deps(pt_wrt_left,  dpl_del);
			jacob_deps_D_p_deps(pt_wrt_right, dpr_der);
			jacob_dA_eps_D_p_deps(lm_stat.right2left_pose, lm_stat.left_cam_poses[k],lm_stat.obj_points[i], dpr_del);

			// Jacobian chain rule:
			dhl_del = dhl_dbl * dbl_dpl * dpl_del;
			dhr_del = dhr_dbr * dbr_dpr * dpr_del;
			dhr_der = dhr_dbr * dbr_dpr * dpr_der;

			rje.J.setZero(4,30);
			rje.J.block<2,6>(0,0) = dhl_del; 
			rje.J.block<2,6>(2,0) = dhr_del; 
			rje.J.block<2,6>(2,6) = dhr_der;
			rje.J.block<2,9>(0,12) = dhl_dcl;
			rje.J.block<2,9>(2,21) = dhr_dcr;
		} // for i
	} // for k

	return total_err;
} // end of recompute_errors_and_Jacobians
