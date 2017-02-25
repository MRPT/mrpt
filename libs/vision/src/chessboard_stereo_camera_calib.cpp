/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "vision-precomp.h"   // Precompiled headers

#include <mrpt/system/filesystem.h>
#include <mrpt/utils/CConfigFileMemory.h>

#include <mrpt/vision/chessboard_find_corners.h>
#include <mrpt/vision/chessboard_stereo_camera_calib.h>
#include <mrpt/vision/pinhole.h>
#include <mrpt/math/robust_kernels.h>
#include <mrpt/math/wrap2pi.h>
#include <algorithm> // reverse()

#include "chessboard_stereo_camera_calib_internal.h"

//#define USE_NUMERIC_JACOBIANS
//#define COMPARE_NUMERIC_JACOBIANS

#ifdef USE_NUMERIC_JACOBIANS
#	include <mrpt/math/jacobians.h>
#endif

using namespace mrpt;
using namespace mrpt::vision;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace std;



/* -------------------------------------------------------
				checkerBoardStereoCalibration
   ------------------------------------------------------- */
bool mrpt::vision::checkerBoardStereoCalibration(
	TCalibrationStereoImageList & images,
	const TStereoCalibParams    & p,
	TStereoCalibResults         & out
	)
{
	try
	{
		ASSERT_(p.check_size_x>2)
		ASSERT_(p.check_size_y>2)
		ASSERT_(p.check_squares_length_X_meters>0)
		ASSERT_(p.check_squares_length_Y_meters>0)

		const bool user_wants_use_robust = p.use_robust_kernel;

		if (images.size()<1)
		{
			std::cout << "ERROR: No input images." << std::endl;
			return false;
		}

		const unsigned   CORNERS_COUNT = p.check_size_x * p.check_size_y;
		const TImageSize check_size    = TImageSize(p.check_size_x, p.check_size_y);
		TImageStereoCallbackData cbPars;

		// For each image, find checkerboard corners:
		// -----------------------------------------------
		// Left/Right sizes (may be different)
        TImageSize imgSize[2] = { TImageSize(0,0), TImageSize(0,0) };

		vector<size_t>   valid_image_pair_indices; // Indices in images[] which are valid pairs to be used in the optimization

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
				// User Callback?
				if (p.callback)
				{
					cbPars.calibRound = -1; // Detecting corners
					cbPars.current_iter = 0;
					cbPars.current_rmse = 0;
					cbPars.nImgsProcessed = i*2 + lr+1;
					cbPars.nImgsToProcess = images.size()*2;
					(*p.callback)(cbPars, p.callback_user_param);
				}

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
			{
				valid_image_pair_indices.push_back(i);

				// Consistency between left/right pair: the corners MUST BE ORDERED so they match to each other,
				// and the criterion must be the same in ALL pairs to avoid rubbish optimization:

				// Key idea: Generate a representative vector that goes along rows and columns.
				// Check the angle of those director vectors between L/R images. That can be done
				// via the dot product. Swap rows/columns order as needed.
				bool has_to_redraw_corners = false;

				const mrpt::math::TPoint2D
					pt_l0 = images[i].left.detected_corners[0],
					pt_l1 = images[i].left.detected_corners[1],
					pt_r0 = images[i].right.detected_corners[0],
					pt_r1 = images[i].right.detected_corners[1];
				const mrpt::math::TPoint2D Al = pt_l1 - pt_l0;
				const mrpt::math::TPoint2D Ar = pt_r1 - pt_r0;

				// If the dot product is negative, we have INVERTED order of corners:
				if (Al.x*Ar.x+Al.y*Ar.y < 0)
				{
					has_to_redraw_corners = true;
					// Invert all corners:
					std::reverse( images[i].right.detected_corners.begin(), images[i].right.detected_corners.end() );
				}


				if (has_to_redraw_corners)
				{
					// Checkboad as color image:
					images[i].right.img_original.colorImage( images[i].right.img_checkboard );
					images[i].right.img_checkboard.drawChessboardCorners( images[i].right.detected_corners,check_size.x,check_size.y);
				}
			}

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
		const double MAX_LAMBDA = 1e20;

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


		// ===============================================================================
		//   Run stereo calibration in two stages:
		//   (0) Estimate all parameters except distortion
		//   (1) Estimate all parameters, using as starting point the guess from (0)
		// ===============================================================================
		size_t nUnknownsCamParams=0;
		size_t iter=0;
		double err=0;
		vector_size_t  vars_to_optimize;
		Eigen::MatrixXd  H;       // Hessian matrix  (Declared here so it's accessible as the final uncertainty measure)

		for (int calibRound=0; calibRound<2;calibRound++)
		{
			cbPars.calibRound = calibRound;
			if (p.verbose)
				cout << ((calibRound==0) ?
				"LM calibration round #0: WITHOUT distortion ----------\n"
				:
				"LM calibration round #1: ALL parameters --------------\n");

			// Select which cam. parameters to optimize (for each camera!) and which to leave fixed:
			// Indices:  [0   1  2  3  4  5  6  7  8]
			// Variables:[fx fy cx cy k1 k2 k3 t1 t2]
			vars_to_optimize.clear();
			vars_to_optimize.push_back(0);
			vars_to_optimize.push_back(1);
			vars_to_optimize.push_back(2);
			vars_to_optimize.push_back(3);
			if (calibRound==1)
			{
				if (p.optimize_k1) vars_to_optimize.push_back(4);
				if (p.optimize_k2) vars_to_optimize.push_back(5);
				if (p.optimize_t1) vars_to_optimize.push_back(6);
				if (p.optimize_t2) vars_to_optimize.push_back(7);
				if (p.optimize_k3) vars_to_optimize.push_back(8);
			}

			nUnknownsCamParams = vars_to_optimize.size();

			//  * N x Manifold Epsilon of left camera pose (6)
			//  * Manifold Epsilon of right2left pose (6)
			//  * Left-cam-params (<=9)
			//  * Right-cam-params (<=9)
			const size_t nUnknowns = N*6 + 6+ 2*nUnknownsCamParams;

			// Residuals & Jacobians:
			TResidualJacobianList res_jacob;
			err = recompute_errors_and_Jacobians(lm_stat,res_jacob, false /* no robust */, p.robust_kernel_param);

			// Build linear system:
			Eigen::VectorXd  minus_g; // minus gradient
			build_linear_system(res_jacob,vars_to_optimize,minus_g,H);

			ASSERT_EQUAL_(nUnknowns,(size_t) H.cols())

			// Lev-Marq. parameters:
			double nu = 2;
			double lambda = tau * H.diagonal().array().maxCoeff();
			bool   done = (minus_g.array().abs().maxCoeff() < t1);
			int    numItersImproving = 0;
			bool   use_robust = false;

			// Lev-Marq. main loop:
			iter=0;
			while (iter<p.maxIters && !done)
			{
				// User Callback?
				if (p.callback)
				{
					cbPars.current_iter = iter;
					cbPars.current_rmse = std::sqrt(err/nObs);
					(*p.callback)(cbPars, p.callback_user_param);
				}

				// Solve for increment: (H + \lambda I) eps = -gradient
				Eigen::MatrixXd  HH = H;
				for (size_t i=0;i<nUnknowns;i++)
					HH(i,i)+=lambda;
					//HH(i,i)*= (1.0 + lambda);

				Eigen::LLT<Eigen::MatrixXd> llt( HH.selfadjointView<Eigen::Lower>() );
				if (llt.info()!=Eigen::Success)
				{
					lambda *= nu;
					nu *= 2;
					done = (lambda>MAX_LAMBDA);

					if (p.verbose && !done) cout << "LM iter#"<<iter<<": Couldn't solve LLt, retrying with larger lambda=" << lambda << endl;
					continue;
				}
				const Eigen::VectorXd eps = llt.solve(minus_g);

				const double eps_norm = eps.norm();
				if (eps_norm < t2*(eps_norm+t2))
				{
					if (p.verbose) cout << "Termination criterion: eps_norm < t2*(eps_norm+t2): "<<eps_norm << " < " << t2*(eps_norm+t2) <<endl;
					done=true;
					break;
				}

				// Tentative new state:
				lm_stat_t new_lm_stat(lm_stat); //images,valid_image_pair_indices,obj_points);
				add_lm_increment(eps,vars_to_optimize, new_lm_stat);

				TResidualJacobianList new_res_jacob;

				// discriminant:
				double err_new = recompute_errors_and_Jacobians(new_lm_stat, new_res_jacob, use_robust, p.robust_kernel_param);

				if(err_new<err)
				{
					//const double l = (err-err_new)/ (eps.dot( lambda*eps + minus_g) );

					if (numItersImproving++>5)
						use_robust  = user_wants_use_robust;

					// Good: Accept new values
					if (p.verbose) cout << "LM iter#"<<iter<<": Avr.err(px):"<< std::sqrt(err/nObs)  <<"->" <<  std::sqrt(err_new/nObs) << " lambda:" << lambda << endl;

					// swap: faster than "lm_stat <- new_lm_stat"
					lm_stat.swap( new_lm_stat );
					res_jacob.swap( new_res_jacob );

					err = err_new;
					build_linear_system(res_jacob,vars_to_optimize,minus_g,H);

					// Too small gradient?
					done = (minus_g.array().abs().maxCoeff() < t1);
					//lambda *= max(1.0/3.0, 1-std::pow(2*l-1,3.0) );
					lambda*=0.6;
					lambda = std::max(lambda, 1e-100);
					nu = 2.0;

					iter++;
				}
				else
				{
					lambda *= nu;
					if (p.verbose) cout << "LM iter#"<<iter<<": No update: err=" << err << " -> err_new=" << err_new << " retrying with larger lambda=" << lambda << endl;
					nu *= 2.0;
					done = (lambda>MAX_LAMBDA);
				}

			} // end while LevMarq.

		} // end for each calibRound = [0,1]
		// -------------------------------------------------------------------------------
		//  End of Levenberg-Marquardt
		// -------------------------------------------------------------------------------

		// Save final optimum values to the output structure
		out.final_rmse  = std::sqrt(err/nObs);
		out.final_iters = iter;
		out.final_number_good_image_pairs = N;

		// [fx fy cx cy k1 k2 k3 t1 t2]
		out.cam_params.leftCamera.fx( lm_stat.left_cam_params[0] );
		out.cam_params.leftCamera.fy( lm_stat.left_cam_params[1] );
		out.cam_params.leftCamera.cx( lm_stat.left_cam_params[2] );
		out.cam_params.leftCamera.cy( lm_stat.left_cam_params[3] );
		out.cam_params.leftCamera.k1( lm_stat.left_cam_params[4] );
		out.cam_params.leftCamera.k2( lm_stat.left_cam_params[5] );
		out.cam_params.leftCamera.k3( lm_stat.left_cam_params[6] );
		out.cam_params.leftCamera.p1( lm_stat.left_cam_params[7] );
		out.cam_params.leftCamera.p2( lm_stat.left_cam_params[8] );


		// [fx fy cx cy k1 k2 k3 t1 t2]
		out.cam_params.rightCamera.fx( lm_stat.right_cam_params[0] );
		out.cam_params.rightCamera.fy( lm_stat.right_cam_params[1] );
		out.cam_params.rightCamera.cx( lm_stat.right_cam_params[2] );
		out.cam_params.rightCamera.cy( lm_stat.right_cam_params[3] );
		out.cam_params.rightCamera.k1( lm_stat.right_cam_params[4] );
		out.cam_params.rightCamera.k2( lm_stat.right_cam_params[5] );
		out.cam_params.rightCamera.k3( lm_stat.right_cam_params[6] );
		out.cam_params.rightCamera.p1( lm_stat.right_cam_params[7] );
		out.cam_params.rightCamera.p2( lm_stat.right_cam_params[8] );

		// R2L pose:
		out.right2left_camera_pose = lm_stat.right2left_pose;

		// All the estimated camera poses:
		out.left_cam_poses = lm_stat.left_cam_poses;

		out.image_pair_was_used.assign(images.size(), false);
		for (size_t i=0;i<valid_image_pair_indices.size();i++)
			out.image_pair_was_used[valid_image_pair_indices[i]]=true;

		// Uncertainties ---------------------
		// The order of inv. variances in the diagonal of the Hessian is:
		//  * N x Manifold Epsilon of left camera pose (6)
		//  * Manifold Epsilon of right2left pose (6)
		//  * Left-cam-params (<=9)
		//  * Right-cam-params (<=9)
		out.left_params_inv_variance.setConstant(0);
		out.right_params_inv_variance.setConstant(0);
		const size_t base_idx_H_CPs = H.cols()-2*nUnknownsCamParams;
		for (size_t i=0;i<nUnknownsCamParams;i++)
		{
			out.left_params_inv_variance [ vars_to_optimize[i] ] = H(base_idx_H_CPs+i,base_idx_H_CPs+i);
			out.right_params_inv_variance[ vars_to_optimize[i] ] = H(base_idx_H_CPs+nUnknownsCamParams+i,base_idx_H_CPs+nUnknownsCamParams+i);
		}

		// Draw projected points
		for (size_t i=0;i<valid_image_pair_indices.size();i++)
		{
			//mrpt::poses::CPose3D			reconstructed_camera_pose;   //!< At output, the reconstructed pose of the camera.
			//std::vector<TPixelCoordf>		projectedPoints_distorted;   //!< At output, only will have an empty vector if the checkerboard was not found in this image, or the predicted (reprojected) corners, which were used to estimate the average square error.
			//std::vector<TPixelCoordf>		projectedPoints_undistorted; //!< At output, like projectedPoints_distorted but for the undistorted image.
			const size_t idx = valid_image_pair_indices[i];

			TImageCalibData &dat_l = images[idx].left;
			TImageCalibData &dat_r = images[idx].right;

			// Rectify image.
			dat_l.img_original.colorImage( dat_l.img_rectified );
			dat_r.img_original.colorImage( dat_r.img_rectified );

			// Camera poses:
			dat_l.reconstructed_camera_pose = - lm_stat.left_cam_poses[idx];
			dat_r.reconstructed_camera_pose = -(lm_stat.right2left_pose + lm_stat.left_cam_poses[idx]);

			// Project distorted images:
			mrpt::vision::pinhole::projectPoints_with_distortion(obj_points, out.cam_params.leftCamera,  CPose3DQuat(dat_l.reconstructed_camera_pose), dat_l.projectedPoints_distorted, true);
			mrpt::vision::pinhole::projectPoints_with_distortion(obj_points, out.cam_params.rightCamera, CPose3DQuat(dat_r.reconstructed_camera_pose), dat_r.projectedPoints_distorted, true);

			// Draw corners:
			dat_l.img_rectified.drawChessboardCorners( dat_l.projectedPoints_distorted, check_size.x,check_size.y);
			dat_r.img_rectified.drawChessboardCorners( dat_r.projectedPoints_distorted, check_size.x,check_size.y);
		}

		return true;
	}
	catch(std::exception &e)
	{
		std::cout << e.what() << std::endl;
		return false;
	}
}

/*
Jacobian:

 d b( [px py pz] )
------------------  (5x3) =
  d{ px py pz }

\left(\begin{array}{ccc} \frac{1}{\mathrm{pz}} & 0 & -\frac{\mathrm{px}}{{\mathrm{pz}}^2}\\ 0 & \frac{1}{\mathrm{pz}} & -\frac{\mathrm{py}}{{\mathrm{pz}}^2} \end{array}\right)

*/

void jacob_db_dp(
	const TPoint3D &p,   // 3D coordinates wrt the camera
	Eigen::Matrix<double,2,3> &G)
{
	const double pz_ = 1/p.z;
	const double pz_2 = pz_*pz_;

	G(0,0) = pz_;
	G(0,1) = 0;
	G(0,2) = -p.x * pz_2;

	G(1,0) = 0;
	G(1,1) = pz_;
	G(1,2) = -p.y * pz_2;
}

/*
Jacobian:

 dh( b,c )
----------- = Hb  | 2x2  (b=[x=px/pz y=py/pz])
  d{ b }

\left(\begin{array}{cc} \mathrm{fx}\, \left(\mathrm{k2}\, {\left(x^2 + y^2\right)}^2 + \mathrm{k3}\, {\left(x^2 + y^2\right)}^3 + 6\, \mathrm{t2}\, x + 2\, \mathrm{t1}\, y + x\, \left(2\, \mathrm{k1}\, x + 4\, \mathrm{k2}\, x\, \left(x^2 + y^2\right) + 6\, \mathrm{k3}\, x\, {\left(x^2 + y^2\right)}^2\right) + \mathrm{k1}\, \left(x^2 + y^2\right) + 1\right) & \mathrm{fx}\, \left(2\, \mathrm{t1}\, x + 2\, \mathrm{t2}\, y + x\, \left(2\, \mathrm{k1}\, y + 4\, \mathrm{k2}\, y\, \left(x^2 + y^2\right) + 6\, \mathrm{k3}\, y\, {\left(x^2 + y^2\right)}^2\right)\right)\\ \mathrm{fy}\, \left(2\, \mathrm{t1}\, x + 2\, \mathrm{t2}\, y + y\, \left(2\, \mathrm{k1}\, x + 4\, \mathrm{k2}\, x\, \left(x^2 + y^2\right) + 6\, \mathrm{k3}\, x\, {\left(x^2 + y^2\right)}^2\right)\right) & \mathrm{fy}\, \left(\mathrm{k2}\, {\left(x^2 + y^2\right)}^2 + \mathrm{k3}\, {\left(x^2 + y^2\right)}^3 + 2\, \mathrm{t2}\, x + 6\, \mathrm{t1}\, y + y\, \left(2\, \mathrm{k1}\, y + 4\, \mathrm{k2}\, y\, \left(x^2 + y^2\right) + 6\, \mathrm{k3}\, y\, {\left(x^2 + y^2\right)}^2\right) + \mathrm{k1}\, \left(x^2 + y^2\right) + 1\right) \end{array}\right)

 dh( b,c )
----------- = Hc  | 2x9
  d{ c }

\left(\begin{array}{ccccccccc} \mathrm{t2}\, \left(3\, x^2 + y^2\right) + x\, \left(\mathrm{k2}\, {\left(x^2 + y^2\right)}^2 + \mathrm{k3}\, {\left(x^2 + y^2\right)}^3 + \mathrm{k1}\, \left(x^2 + y^2\right) + 1\right) + 2\, \mathrm{t1}\, x\, y & 0 & 1 & 0 & \mathrm{fx}\, x\, \left(x^2 + y^2\right) & \mathrm{fx}\, x\, {\left(x^2 + y^2\right)}^2 & \mathrm{fx}\, x\, {\left(x^2 + y^2\right)}^3 & 2\, \mathrm{fx}\, x\, y & \mathrm{fx}\, \left(3\, x^2 + y^2\right)\\ 0 & \mathrm{t1}\, \left(x^2 + 3\, y^2\right) + y\, \left(\mathrm{k2}\, {\left(x^2 + y^2\right)}^2 + \mathrm{k3}\, {\left(x^2 + y^2\right)}^3 + \mathrm{k1}\, \left(x^2 + y^2\right) + 1\right) + 2\, \mathrm{t2}\, x\, y & 0 & 1 & \mathrm{fy}\, y\, \left(x^2 + y^2\right) & \mathrm{fy}\, y\, {\left(x^2 + y^2\right)}^2 & \mathrm{fy}\, y\, {\left(x^2 + y^2\right)}^3 & \mathrm{fy}\, \left(x^2 + 3\, y^2\right) & 2\, \mathrm{fy}\, x\, y \end{array}\right)

*/

void jacob_dh_db_and_dh_dc(
	const TPoint3D & nP,  // Point in relative coords wrt the camera
	const Eigen::Matrix<double,9,1>  & c,  // camera parameters
	Eigen::Matrix<double,2,2>  & Hb,
	Eigen::Matrix<double,2,9> & Hc
	)
{
	const double x = nP.x/nP.z;
	const double y = nP.y/nP.z;

	const double r2   = x*x + y*y;
	const double r    = std::sqrt(r2);
	const double r6   = r2*r2*r2; // (x^2+y^2)^3 = r^6

	// c=[fx fy cx cy k1 k2 k3 t1 t2]
	const double fx=c[0], fy=c[1];
	//const double cx=c[2], cy=c[3]; // Un-unused
	const double k1=c[4], k2=c[5], k3=c[6];
	const double t1=c[7], t2=c[8];

	// Hb = dh(b,c)/db  (2x2)
	Hb(0,0) = fx*(k2*r2 + k3*r6 + 6*t2*x + 2*t1*y + x*(2*k1*x + 4*k2*x*r + 6*k3*x*r2) + k1*r + 1);
	Hb(0,1) = fx*(2*t1*x + 2*t2*y + x*(2*k1*y + 4*k2*y*r + 6*k3*y*r2));

	Hb(1,0) = fy*(2*t1*x + 2*t2*y + y*(2*k1*x + 4*k2*x*r + 6*k3*x*r2));
	Hb(1,1) = fy*(k2*r2 + k3*r6 + 2*t2*x + 6*t1*y + y*(2*k1*y + 4*k2*y*r + 6*k3*y*r2) + k1*r + 1);


	// Hc = dh(b,c)/dc  (2x9)
	Hc(0,0) = t2*(3*x*x + y*y) + x*(k2*r2 + k3*r6 + k1*r + 1) + 2*t1*x*y;
	Hc(0,1) = 0;
	Hc(0,2) = 1;
	Hc(0,3) = 0;
	Hc(0,4) = fx*x*r;
	Hc(0,5) = fx*x*r2;
	Hc(0,6) = fx*x*r6;
	Hc(0,7) = 2*fx*x*y;
	Hc(0,8) = fx*(3*x*x + y*y);

	Hc(1,0) = 0;
	Hc(1,1) = t1*(x*x + 3*y*y) + y*(k2*r2 + k3*r6 + k1*r + 1) + 2*t2*x*y;
	Hc(1,2) = 0;
	Hc(1,3) = 1;
	Hc(1,4) = fy*y*r;
	Hc(1,5) = fy*y*r2;
	Hc(1,6) = fy*y*r6;
	Hc(1,7) = fy*(x*x + 3*y*y);
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
	H.block<3,3>(0,3) = mrpt::math::skew_symmetric3_neg(v);

	dp_deps.noalias() = A.getRotationMatrix() * H;
}


void project_point(
	const mrpt::math::TPoint3D & P,
	const mrpt::utils::TCamera & params,
	const CPose3D              & cameraPose,
	mrpt::utils::TPixelCoordf & px
	)
{
	// Change the reference system to that wrt the camera
	TPoint3D nP;
	cameraPose.composePoint( P.x, P.y, P.z, nP.x, nP.y, nP.z );

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
//  * Left-cam-params (<=9)
//  * Right-cam-params (<=9)
void mrpt::vision::build_linear_system(
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
	// Also include additional cost terms to stabilize estimation:
	// -----------------------------------------------------------------
	// In the left->right pose increment: Add cost if we know that both cameras are almost parallel:
	const double cost_lr_angular = 1e10;
	H_tot.block<3,3>(N*6+3,N*6+3) += Eigen::Matrix<double,3,3>::Identity() * cost_lr_angular;
#endif

	// Ignore those derivatives of "fixed" (non-optimizable) variables in camera parameters
	// --------------------------------------------------------------------------------------
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
	}

	for (size_t k=0;k<nUnkPoses;k++)
	{
		for (size_t i=0;i<N_Cs;i++)
		{
			H(nUnkPoses+i,k) = H(k,nUnkPoses+i) = H_tot(k,nUnkPoses+var_indxs[i]);
			H(nUnkPoses+i+N_Cs,k) = H(k,nUnkPoses+i+N_Cs) = H_tot(k,nUnkPoses+9+var_indxs[i]);
		}
	}

	for (size_t i=0;i<N_Cs;i++)
	{
		for (size_t j=0;j<N_Cs;j++)
		{
			H(nUnkPoses+i,nUnkPoses+j) = H_tot( nUnkPoses + var_indxs[i], nUnkPoses + var_indxs[j] );
			H(nUnkPoses+N_Cs+i,nUnkPoses+N_Cs+j) = H_tot( nUnkPoses +9+var_indxs[i], nUnkPoses +9+ var_indxs[j] );
		}
	}

#if 0
	{
		CMatrixDouble M1 = H_tot;
		CMatrixDouble g1 = minus_g_tot;
		M1.saveToTextFile("H1.txt");
		g1.saveToTextFile("g1.txt");

		CMatrixDouble M2 = H;
		CMatrixDouble g2 = minus_g;
		M2.saveToTextFile("H2.txt");
		g2.saveToTextFile("g2.txt");
		mrpt::system::pause();
	}
#endif

} // end of build_linear_system


//  * N x Manifold Epsilon of left camera pose (6)
//  * Manifold Epsilon of right2left pose (6)
//  * Left-cam-params (<=9)
//  * Right-cam-params (<=9)
void mrpt::vision::add_lm_increment(
	const Eigen::VectorXd & eps,
	const vector_size_t   & var_indxs,
	lm_stat_t             & lm_stat)
{
	// Increment of the N cam poses
	const size_t N = lm_stat.valid_image_pair_indices.size();
	for (size_t i=0;i<N;i++)
	{
		CPose3D &cam_pose = lm_stat.left_cam_poses[ lm_stat.valid_image_pair_indices[i] ];

		// Use the Lie Algebra methods for the increment:
		const CArrayDouble<6> incr( &eps[6*i] );
		const CPose3D         incrPose = CPose3D::exp(incr);

		//new_pose =  old_pose  [+] delta
		//         = exp(delta) (+) old_pose
		cam_pose.composeFrom(incrPose, cam_pose );
	}

	// Increment of the right-left pose:
	{
		// Use the Lie Algebra methods for the increment:
		const CArrayDouble<6> incr( &eps[6*N] );
		const CPose3D         incrPose = CPose3D::exp(incr);

		//new_pose =  old_pose  [+] delta
		//         = exp(delta) (+) old_pose
		lm_stat.right2left_pose.composeFrom(incrPose, lm_stat.right2left_pose );
	}

	// Increment of the camera params:
	const size_t idx=6*N+6;
	const size_t nPC = var_indxs.size();  // Number of camera parameters being optimized
	for (size_t i=0;i<nPC;i++)
	{
		lm_stat.left_cam_params [var_indxs[i]] += eps[idx+i];
		lm_stat.right_cam_params[var_indxs[i]] += eps[idx+nPC+i];
	}

} // end of add_lm_increment


#ifdef USE_NUMERIC_JACOBIANS
// ---------------------------------------------------------------
// Aux. function, only if we evaluate Jacobians numerically:
// ---------------------------------------------------------------
	struct TNumJacobData
	{
		const lm_stat_t & lm_stat;
		const TPoint3D  & obj_point;
		const CPose3D   & left_cam;
		const CPose3D   & right2left;
		const Eigen::Matrix<double,4,1> & real_obs;

		TNumJacobData(
			const lm_stat_t & _lm_stat,
			const TPoint3D  & _obj_point,
			const CPose3D   & _left_cam,
			const CPose3D   & _right2left,
			const Eigen::Matrix<double,4,1> &_real_obs
			) : lm_stat(_lm_stat), obj_point(_obj_point), left_cam(_left_cam), right2left(_right2left), real_obs(_real_obs)
		{
		}

	};

	void numeric_jacob_eval_function(
		const CArrayDouble<30> &x,
		const TNumJacobData &dat,
		CArrayDouble<4> &out)
	{
		// Recover the state out from "x":
		const CArrayDouble<6> incr_l( &x[0] );
		const CPose3D         incrPose_l = CPose3D::exp(incr_l);
		const CArrayDouble<6> incr_rl( &x[6] );
		const CPose3D         incrPose_rl = CPose3D::exp(incr_rl);

		// [fx fy cx cy k1 k2 k3 t1 t2]
		TStereoCamera cam_params;
		CArrayDouble<9> left_cam_params = x.segment<9>(6+6);
		cam_params.leftCamera.fx( left_cam_params[0] );
		cam_params.leftCamera.fy( left_cam_params[1] );
		cam_params.leftCamera.cx( left_cam_params[2] );
		cam_params.leftCamera.cy( left_cam_params[3] );
		cam_params.leftCamera.k1( left_cam_params[4] );
		cam_params.leftCamera.k2( left_cam_params[5] );
		cam_params.leftCamera.k3( left_cam_params[6] );
		cam_params.leftCamera.p1( left_cam_params[7] );
		cam_params.leftCamera.p2( left_cam_params[8] );

		// [fx fy cx cy k1 k2 k3 t1 t2]
		CArrayDouble<9> right_cam_params = x.segment<9>(6+6+9);
		cam_params.rightCamera.fx( right_cam_params[0] );
		cam_params.rightCamera.fy( right_cam_params[1] );
		cam_params.rightCamera.cx( right_cam_params[2] );
		cam_params.rightCamera.cy( right_cam_params[3] );
		cam_params.rightCamera.k1( right_cam_params[4] );
		cam_params.rightCamera.k2( right_cam_params[5] );
		cam_params.rightCamera.k3( right_cam_params[6] );
		cam_params.rightCamera.p1( right_cam_params[7] );
		cam_params.rightCamera.p2( right_cam_params[8] );

		TPixelCoordf px_l;
		project_point( dat.obj_point,cam_params.leftCamera, incrPose_l + dat.left_cam, px_l );
		TPixelCoordf px_r;
		project_point( dat.obj_point,cam_params.rightCamera, incrPose_rl + dat.right2left + incrPose_l + dat.left_cam, px_r );

		const Eigen::Matrix<double,4,1> predicted_obs( px_l.x,px_l.y, px_r.x,px_r.y );

		// Residual:
		out = predicted_obs; // - dat.real_obs;
	}

	void eval_h_b(
		const CArrayDouble<2> &X,
		const TCamera &params,
		CArrayDouble<2> &out)
	{
		// Radial distortion:
		const double x = X[0], y=X[1];
		const double r2 = square(x)+square(y);
		const double r4 = square(r2);
		const double r6 = r2*r4;
		const double A  = 1+params.dist[0]*r2+params.dist[1]*r4+params.dist[4]*r6;
		const double B  = 2*x*y;

		out[0] = params.cx() + params.fx() * ( x*A + params.dist[2]*B + params.dist[3]*(r2+2*square(x)) );
		out[1] = params.cy() + params.fy() * ( y*A + params.dist[3]*B + params.dist[2]*(r2+2*square(y)) );
	}

	void eval_b_p(
		const CArrayDouble<3> &P,
		const int &dummy,
		CArrayDouble<2> &b)
	{
		// Radial distortion:
		b[0] = P[0]/P[2];
		b[1] = P[1]/P[2];
	}


	void eval_deps_D_p(
		const CArrayDouble<6> &eps,
		const TPoint3D &D_p,
		CArrayDouble<3> &out)
	{
		const CArrayDouble<6> incr( &eps[0] );
		const CPose3D         incrPose = CPose3D::exp(incr);
		TPoint3D D_p_out;
		incrPose.composePoint(D_p,D_p_out);
		for (int i=0;i<3;i++) out[i]=D_p_out[i];
	}

	struct TEvalData_A_eps_D_p
	{
		CPose3D  A, D;
		TPoint3D p;
	};

	void eval_dA_eps_D_p(
		const CArrayDouble<6> &eps,
		const TEvalData_A_eps_D_p  &dat,
		CArrayDouble<3> &out)
	{
		const CArrayDouble<6> incr( &eps[0] );
		const CPose3D  incrPose = CPose3D::exp(incr);
		const CPose3D  A_eps_D = dat.A + (incrPose + dat.D);
		TPoint3D pt;
		A_eps_D.composePoint(dat.p,pt);
		for (int i=0;i<3;i++) out[i]=pt[i];
	}
// ---------------------------------------------------------------
// End of aux. function, only if we evaluate Jacobians numerically:
// ---------------------------------------------------------------
#endif





// the 4x1 prediction are the (u,v) pixel coordinates for the left / right cameras:
double mrpt::vision::recompute_errors_and_Jacobians(
	const lm_stat_t       & lm_stat,
	TResidualJacobianList & res_jac,
	bool use_robust_kernel,
	double kernel_param )
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
			project_point( lm_stat.obj_points[i], camparam_l, lm_stat.left_cam_poses[k_idx], px_l );
			project_point( lm_stat.obj_points[i], camparam_r, lm_stat.right2left_pose+lm_stat.left_cam_poses[k_idx], px_r );
			rje.predicted_obs = Eigen::Matrix<double,4,1>( px_l.x,px_l.y, px_r.x,px_r.y );

			// Residual:
			rje.residual = rje.predicted_obs - obs;

			// Accum. total squared error:
			const double err_sqr_norm = rje.residual.squaredNorm();
			if (use_robust_kernel)
			{
				RobustKernel<rkPseudoHuber>  rk;
				rk.param_sq = kernel_param;

				double kernel_1st_deriv,kernel_2nd_deriv;
				const double scaled_err = rk.eval(err_sqr_norm, kernel_1st_deriv,kernel_2nd_deriv );

				rje.residual *= kernel_1st_deriv;
				total_err += scaled_err;
			}
			else	total_err += err_sqr_norm;

			// ---------------------------------------------------------------------------------
			// Jacobian: (4x30)
			// See technical report cited in the headers for the theory behind these formulas.
			// ---------------------------------------------------------------------------------
#if !defined(USE_NUMERIC_JACOBIANS) || defined(COMPARE_NUMERIC_JACOBIANS)
			// ----- Theoretical Jacobians -----
			Eigen::Matrix<double,2,6> dhl_del, dhr_del, dhr_der;
			Eigen::Matrix<double,2,9> dhl_dcl, dhr_dcr;

			// 3D coordinates of the corner point wrt both cameras:
			TPoint3D pt_wrt_left, pt_wrt_right;
			lm_stat.left_cam_poses[k_idx].composePoint(lm_stat.obj_points[i], pt_wrt_left );
			(lm_stat.right2left_pose+lm_stat.left_cam_poses[k_idx]).composePoint(lm_stat.obj_points[i], pt_wrt_right);

			// Build partial Jacobians:
			Eigen::Matrix<double,2,2> dhl_dbl, dhr_dbr;
			jacob_dh_db_and_dh_dc(pt_wrt_left,  lm_stat.left_cam_params,  dhl_dbl, dhl_dcl );
			jacob_dh_db_and_dh_dc(pt_wrt_right, lm_stat.right_cam_params, dhr_dbr, dhr_dcr );

#if 0
			// Almost exact....
			{
				CArrayDouble<2> x0;
				TPoint3D nP = pt_wrt_left;
				x0[0] = nP.x/nP.z;
				x0[1] = nP.y/nP.z;

				CArrayDouble<2> x_incrs;
				x_incrs.setConstant(1e-6);

				Eigen::Matrix<double,2,2> num_dhl_dbl, num_dhr_dbr;
				mrpt::math::jacobians::jacob_numeric_estimate(x0, &eval_h_b, x_incrs, camparam_l, num_dhl_dbl );

				nP = pt_wrt_right;
				x0[0] = nP.x/nP.z;
				x0[1] = nP.y/nP.z;
				mrpt::math::jacobians::jacob_numeric_estimate(x0, &eval_h_b, x_incrs, camparam_r, num_dhr_dbr );

				cout << "num_dhl_dbl:\n" << num_dhl_dbl << "\ndiff dhl_dbl:\n" << dhl_dbl-num_dhl_dbl << endl << endl;
				cout << "num_dhr_dbr:\n" << num_dhr_dbr << "\ndiff dhr_dbr:\n" << dhr_dbr-num_dhr_dbr << endl << endl;

			}
#endif

			Eigen::Matrix<double,2,3> dbl_dpl, dbr_dpr;
			jacob_db_dp(pt_wrt_left,  dbl_dpl);
			jacob_db_dp(pt_wrt_right, dbr_dpr);

#if 0
			// OK! 100% exact.
			{
				CArrayDouble<3> x0;
				x0[0]=pt_wrt_left.x;
				x0[1]=pt_wrt_left.y;
				x0[2]=pt_wrt_left.z;

				CArrayDouble<3> x_incrs;
				x_incrs.setConstant(1e-8);

				Eigen::Matrix<double,2,3> num_dbl_dpl, num_dbr_dpr;
				const int dumm=0;
				mrpt::math::jacobians::jacob_numeric_estimate(x0, &eval_b_p, x_incrs, dumm, num_dbl_dpl );

				x0[0]=pt_wrt_right.x;
				x0[1]=pt_wrt_right.y;
				x0[2]=pt_wrt_right.z;
				mrpt::math::jacobians::jacob_numeric_estimate(x0, &eval_b_p, x_incrs, dumm, num_dbr_dpr );

				cout << "num_dbl_dpl:\n" << num_dbl_dpl << "\ndbl_dpl:\n" << dbl_dpl << endl << endl;
				cout << "num_dbr_dpr:\n" << num_dbr_dpr << "\ndbr_dpr:\n" << dbr_dpr << endl << endl;

			}
#endif

			// p_l = exp(epsilon_l) (+) pose_left (+) point_ij
			// p_l = [exp(epsilon_r) (+) pose_right2left] (+) [exp(epsilon_l) (+) pose_left] (+) point_ij
			Eigen::Matrix<double,3,6> dpl_del, dpr_del, dpr_der;
			jacob_deps_D_p_deps(pt_wrt_left,  dpl_del);
			jacob_deps_D_p_deps(pt_wrt_right, dpr_der);
			jacob_dA_eps_D_p_deps(lm_stat.right2left_pose, lm_stat.left_cam_poses[k_idx],lm_stat.obj_points[i], dpr_del);

#if 0
			// 100% Exact.
			{
				// Test jacob_deps_D_p_deps:
				CArrayDouble<6> x0;
				x0.setConstant(0);

				CArrayDouble<6> x_incrs;
				x_incrs.setConstant(1e-8);

				Eigen::Matrix<double,3,6> num_dpl_del, num_dpr_der;
				mrpt::math::jacobians::jacob_numeric_estimate(x0, &eval_deps_D_p, x_incrs, pt_wrt_left , num_dpl_del );
				mrpt::math::jacobians::jacob_numeric_estimate(x0, &eval_deps_D_p, x_incrs, pt_wrt_right, num_dpr_der );

				cout << "num_dpl_del:\n" << num_dpl_del << "\ndiff dpl_del:\n" << dpl_del-num_dpl_del << endl << endl;
				cout << "num_dpr_der:\n" << num_dpr_der << "\ndiff dpr_der:\n" << dpr_der-num_dpr_der << endl << endl;
			}
#endif

#if 0
			// 100% Exact.
			{
				// Test jacob_dA_eps_D_p_deps:
				CArrayDouble<6> x0;
				x0.setConstant(0);

				CArrayDouble<6> x_incrs;
				x_incrs.setConstant(1e-8);

				TEvalData_A_eps_D_p dat;
				dat.A = lm_stat.right2left_pose;
				dat.D = lm_stat.left_cam_poses[k_idx];
				dat.p = lm_stat.obj_points[i];

				Eigen::Matrix<double,3,6> num_dpr_del;
				mrpt::math::jacobians::jacob_numeric_estimate(x0, &eval_dA_eps_D_p, x_incrs,dat , num_dpr_del );

				cout << "num_dpr_del:\n" << num_dpr_del << "\ndiff dpr_del:\n" << num_dpr_del-dpr_del << endl << endl;
			}
#endif

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

#	if defined(COMPARE_NUMERIC_JACOBIANS)
			const Eigen::Matrix<double,4,30> J_theor=rje.J;
#	endif
			// ---- end of theoretical Jacobians ----
#endif

#if defined(USE_NUMERIC_JACOBIANS) || defined(COMPARE_NUMERIC_JACOBIANS)
			// ----- Numeric Jacobians ----

			CArrayDouble<30> x0;  // eps_l (6) + eps_lr (6) + l_camparams (9) + r_camparams (9)
			x0.setZero();
			x0.segment<9>(6+6)   = lm_stat.left_cam_params;
			x0.segment<9>(6+6+9) = lm_stat.right_cam_params;

			const double x_incrs_val[30] =  {
				1e-6,1e-6,1e-6, 1e-7,1e-7,1e-7,  // eps_l
				1e-6,1e-6,1e-6, 1e-7,1e-7,1e-7,  // eps_r
				1e-3,1e-3,1e-3,1e-3, 1e-8,1e-8,1e-8,1e-8, 1e-4,  // cam_l
				1e-3,1e-3,1e-3,1e-3, 1e-8,1e-8,1e-8,1e-8, 1e-4  // cam_rl
			};
			const CArrayDouble<30> x_incrs(x_incrs_val);
			TNumJacobData dat(lm_stat,lm_stat.obj_points[i],lm_stat.left_cam_poses[k_idx], lm_stat.right2left_pose, obs);

			mrpt::math::jacobians::jacob_numeric_estimate(x0, &numeric_jacob_eval_function, x_incrs, dat, rje.J );

#	if defined(COMPARE_NUMERIC_JACOBIANS)
			const Eigen::Matrix<double,4,30> J_num=rje.J;
#	endif
#endif		// ---- end of numeric Jacobians ----

// Only for debugging:
#if defined(COMPARE_NUMERIC_JACOBIANS)
			//if ( (J_num-J_theor).array().abs().maxCoeff()>1e-2)
			{
				ofstream f;
				f.open("dbg.txt", ios_base::out | ios_base::app);
				f << "J_num:\n" << J_num << endl
				  << "J_theor:\n" << J_theor << endl
				  << "diff:\n" << J_num - J_theor << endl
				  << "diff (ratio):\n" << (J_num - J_theor).cwiseQuotient(J_num) << endl << endl;
			}
#endif
		} // for i
	} // for k

	return total_err;
} // end of recompute_errors_and_Jacobians


// Ctor:
TStereoCalibParams::TStereoCalibParams() :
	check_size_x(7),check_size_y(9),
	check_squares_length_X_meters(0.02),check_squares_length_Y_meters(0.02),
	normalize_image(true),
	skipDrawDetectedImgs(false),
	verbose(true),
	maxIters(2000),
	optimize_k1(true), optimize_k2(true), optimize_k3(false),
	optimize_t1(false),optimize_t2(false),
	use_robust_kernel(false),
	robust_kernel_param(10),
	callback(NULL),
	callback_user_param(NULL)
{
}

TStereoCalibResults::TStereoCalibResults() :
	final_rmse (0),
	final_iters(0),
	final_number_good_image_pairs(0)
{
}
