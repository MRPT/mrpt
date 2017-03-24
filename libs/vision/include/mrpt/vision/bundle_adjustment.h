/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef mrpt_vision_ba_H
#define mrpt_vision_ba_H

#include <mrpt/vision/types.h>
#include <mrpt/utils/TCamera.h>
#include <mrpt/utils/TParameters.h>
#include <mrpt/math/lightweight_geom_data.h>
#include <mrpt/utils/CArray.h>

// The methods declared in this file are implemented in separate files in: vision/src/ba_*.cpp
namespace mrpt
{
	namespace vision
	{
		/** \defgroup bundle_adj Bundle-Adjustment methods
		  * \ingroup mrpt_vision_grp
		  */

		/** @name Bundle-Adjustment methods
		    @{ */

		/** A functor type for BA methods \sa bundle_adj_full */
		typedef void (*TBundleAdjustmentFeedbackFunctor)(
			const size_t cur_iter,
			const double cur_total_sq_error,
			const size_t max_iters,
			const mrpt::vision::TSequenceFeatureObservations & input_observations,
			const mrpt::vision::TFramePosesVec & current_frame_estimate,
			const mrpt::vision::TLandmarkLocationsVec & current_landmark_estimate );


		/** Sparse Levenberg-Marquart solution to bundle adjustment - optimizes all the camera frames & the landmark locations.
		  * At input a gross estimation of the frame poses & the landmark points must be supplied. If you don't have such a
		  *  starting point, use mrpt::vision::ba_initial_estimate() to compute it.
		  *
		  * At output the best found solution will be returned in the variables. Optionally, a functor can be passed for having
		  *  feedback on the progress at each iteration (you can use it to refresh a GUI, display a progress bar, etc...).
		  *
		  * This implementation is almost entirely an adapted version from RobotVision (at OpenSLAM.org) (C) by Hauke Strasdat (Imperial College London), licensed under GNU LGPL.
		  *  See the related paper:  H. Strasdat, J.M.M. Montiel, A.J. Davison: "Scale Drift-Aware Large Scale Monocular SLAM", RSS2010, http://www.roboticsproceedings.org/rss06/p10.html
		  *
		  *  List of optional parameters in "extra_params":
		  *		- "verbose" : Verbose output (default=0)
		  *		- "max_iterations": Maximum number of iterations to run (default=50)
		  *		- "robust_kernel": If !=0, use a robust kernel against outliers (default=1)
		  *		- "kernel_param": The pseudo-huber kernel parameter (default=3)
		  *		- "mu": Initial mu for LevMarq (default=-1 -> autoguess)
		  *		- "num_fix_frames": Number of first frame poses to don't optimize (keep unmodified as they come in)  (default=1: the first pose is the reference and is not modified)
		  *		- "num_fix_points": Idem, for the landmarks positions (default=0: optimize all)
		  *		- "profiler": If !=0, displays profiling information to the console at return.
		  *
		  * \note In this function, all coordinates are absolute. Camera frames are such that +Z points forward from the focal point (see the figure in mrpt::obs::CObservationImage).
		  * \note The first frame pose will be not updated since at least one frame must remain fixed.
		  *
		  * \param observations [IN] All the feature observations (WITHOUT distortion), indexed by feature ID as lists of <frame_ID, (x,y)>. See TSequenceFeatureObservations.
		  * \param camera_params [IN] The camera parameters, mainly used for the intrinsic 3x3 matrix. Distortion params are ignored since it's assumed that \a observations are already undistorted pixels.
		  * \param frame_poses [IN/OUT] Input: Gross estimation of each frame poses. Output: The found optimal solution.
		  * \param landmark_points [IN/OUT] Input: Gross estimation of each landmark point. Output: The found optimal solution.
		  * \param extra_params [IN] Optional extra parameters. Read above.
		  * \param user_feedback [IN] If provided, this functor will be called at each iteration to provide a feedback to the user.
		  *
		  * \return The final overall squared error.
		  * \ingroup bundle_adj
		  */
		double VISION_IMPEXP bundle_adj_full(
			const mrpt::vision::TSequenceFeatureObservations   & observations,
			const mrpt::utils::TCamera                        & camera_params,
			mrpt::vision::TFramePosesVec                       & frame_poses,
			mrpt::vision::TLandmarkLocationsVec                & landmark_points,
			const mrpt::utils::TParametersDouble & extra_params = mrpt::utils::TParametersDouble(),
			const mrpt::vision::TBundleAdjustmentFeedbackFunctor user_feedback = NULL
			);


		/** @} */


		/** @name Bundle-Adjustment Auxiliary methods
		    @{ */

		/** Fills the frames & landmark points maps with an initial gross estimate from the sequence \a observations, so they can be fed to bundle adjustment methods.
		  * \sa bundle_adj_full
		  * \ingroup bundle_adj
		  */
		void VISION_IMPEXP ba_initial_estimate(
			const mrpt::vision::TSequenceFeatureObservations   & observations,
			const mrpt::utils::TCamera                        & camera_params,
			mrpt::vision::TFramePosesVec                       & frame_poses,
			mrpt::vision::TLandmarkLocationsVec                & landmark_points );

		//! \overload
		void VISION_IMPEXP ba_initial_estimate(
			const mrpt::vision::TSequenceFeatureObservations   & observations,
			const mrpt::utils::TCamera                        & camera_params,
			mrpt::vision::TFramePosesMap                       & frame_poses,
			mrpt::vision::TLandmarkLocationsMap                & landmark_points );


		/** Compute reprojection error vector (used from within Bundle Adjustment methods, but can be used in general)
		  *  See mrpt::vision::bundle_adj_full for a description of most parameters.
		  * \param frame_poses_are_inverse If set to true, global camera poses are \f$ \ominus F \f$ instead of \f$ F \f$, for each F in frame_poses.
		  *
		  *  \return Overall squared reprojection error.
		  * \ingroup bundle_adj
		  */
		double VISION_IMPEXP reprojectionResiduals(
			const mrpt::vision::TSequenceFeatureObservations   & observations,
			const mrpt::utils::TCamera                        & camera_params,
			const mrpt::vision::TFramePosesVec                 & frame_poses,
			const mrpt::vision::TLandmarkLocationsVec          & landmark_points,
			std::vector<mrpt::utils::CArray<double,2> > & out_residuals,
			const bool  frame_poses_are_inverse,
			const bool  use_robust_kernel = true,
			const double kernel_param = 3.0,
			std::vector<double> * out_kernel_1st_deriv = NULL
			);

		//! \overload
		double VISION_IMPEXP reprojectionResiduals(
			const mrpt::vision::TSequenceFeatureObservations   & observations,
			const mrpt::utils::TCamera                        & camera_params,
			const mrpt::vision::TFramePosesMap                 & frame_poses,
			const mrpt::vision::TLandmarkLocationsMap          & landmark_points,
			std::vector<mrpt::utils::CArray<double,2> > & out_residuals,
			const bool  frame_poses_are_inverse,
			const bool  use_robust_kernel = true,
			const double kernel_param = 3.0,
			std::vector<double> * out_kernel_1st_deriv = NULL
			);


		/** For each pose in the vector \a frame_poses, adds a "delta" increment to the manifold, with the "delta" given in the se(3) Lie algebra:
		  *
		  *    new_frame_poses[i] = frame_poses[i] [+] delta[(first_idx+6*i):(first_idx+6*i+5)]    , for every pose in \a frame_poses
		  *
		  *  With the left-multiplication convention of the manifold exp(delta) operator, that is:
		  *
		  *     p <-- p [+] delta ==>  p <-- exp(delta) * p
		  *
		  * \param delta_num_vals Used just for sanity check, must be equal to "frame_poses.size() * 6"
		  * \ingroup bundle_adj
		  */
		void VISION_IMPEXP add_se3_deltas_to_frames(
			const mrpt::vision::TFramePosesVec & frame_poses,
			const mrpt::math::CVectorDouble &delta,
			const size_t         delta_first_idx,
			const size_t         delta_num_vals,
			mrpt::vision::TFramePosesVec       & new_frame_poses,
			const size_t         num_fix_frames );

		/** For each pose in the vector \a frame_poses, adds a "delta" increment to the manifold, with the "delta" given in the se(3) Lie algebra:
		  *
		  *    new_landmark_points[i] = landmark_points[i] + delta[(first_idx+3*i):(first_idx+3*i+2)]    , for every pose in \a landmark_points
		  *
		  * \param delta_num_vals Used just for sanity check, must be equal to "landmark_points.size() * 3"
		  * \ingroup bundle_adj
		  */
		void VISION_IMPEXP add_3d_deltas_to_points(
			const mrpt::vision::TLandmarkLocationsVec & landmark_points,
			const mrpt::math::CVectorDouble       & delta,
			const size_t                delta_first_idx,
			const size_t                delta_num_vals,
			mrpt::vision::TLandmarkLocationsVec        & new_landmark_points,
			const size_t                num_fix_points );

		/** @} */
	}
}
#endif

