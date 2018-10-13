/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/img/CImage.h>
#include <mrpt/img/TStereoCamera.h>
#include <mrpt/vision/types.h>
#include <mrpt/vision/chessboard_camera_calib.h>

namespace mrpt::vision
{
/** \addtogroup chessboard_calib Chessboard calibration
 *  \ingroup mrpt_vision_grp
 *  @{  */

/** Data associated to each stereo image in the calibration process
 * mrpt::vision::checkerBoardCameraCalibration (All the information can be left
 * empty and will be filled up in the calibration method).
 */
struct TImageStereoCalibData
{
	TImageCalibData left, right;

	/** Empty all the data */
	void clear() { *this = TImageStereoCalibData(); }
};

/** Params of the optional callback provided by the user */
struct TImageStereoCallbackData
{
	/** =-1:Processing images;  =0: Initial calib without distortion, =1: Calib
	 * of all parameters */
	int calibRound;
	size_t current_iter;
	/** Current root-mean square reprojection error (in pixels) */
	double current_rmse;
	/** Info for calibRound==-1 */
	unsigned int nImgsProcessed, nImgsToProcess;
};

/** Prototype of optional user callback function. */
using TSteroCalibCallbackFunctor =
	void (*)(const TImageStereoCallbackData& d, void* user_data);

/** Input parameters for mrpt::vision::checkerBoardStereoCalibration */
struct TStereoCalibParams
{
	/** The number of squares in the checkerboard in the "X" & "Y" direction. */
	unsigned int check_size_x{7}, check_size_y{9};
	/** The size of each square in the checkerboard, in meters, in the "X" & Y"
	 * axes. */
	double check_squares_length_X_meters{0.02},
		check_squares_length_Y_meters{0.02};
	bool normalize_image{true};
	bool skipDrawDetectedImgs{false};
	/** Show progress messages to std::cout console (default=true) */
	bool verbose{true};
	/** Maximum number of iterations of the optimizer (default=300) */
	size_t maxIters{2000};

	/** Select which distortion parameters (of both left/right cameras) will be
	 * optimzed:
	 *  k1,k2,k3 are the r^2, r^4 and r^6 radial distorion coeficients, and t1
	 * and t2 are the tangential distortion coeficients (see
	 * mrpt::img::TCamera).
	 * Those set to false will be assumed to be fixed to zero (no distortion).
	 * \note Default values are to only assume distortion via k1 and k2 (the
	 * rest are zeros).
	 */
	bool optimize_k1{true}, optimize_k2{true}, optimize_k3{false},
		optimize_t1{false}, optimize_t2{false};

	/** Employ a Pseudo-Huber robustifier kernel (Default: false) */
	bool use_robust_kernel{false};
	/** The parameter of the robust kernel, in pixels (only if
	 * use_robust_kernel=true) (Default=10) */
	double robust_kernel_param{10};

	/** If set to !=NULL, this function will be called within each Lev-Marq.
	 * iteration (don't do heavy stuff here since performance will degrade) */
	TSteroCalibCallbackFunctor callback{nullptr};
	/** If using a callback function, you can use this to pass custom data to
	 * your callback. */
	void* callback_user_param{nullptr};

	// Ctor: Set default values
	TStereoCalibParams();
};

/** Output results for mrpt::vision::checkerBoardStereoCalibration */
struct TStereoCalibResults
{
	TStereoCalibResults();

	/** Recovered parameters of the stereo camera */
	mrpt::img::TStereoCamera cam_params;
	/** The pose of the left camera as seen from the right camera */
	mrpt::poses::CPose3D right2left_camera_pose;

	/** Poses of the origin of coordinates of the pattern wrt the left camera
	 * (i.e. the origin of coordinates, as seen from the different camera poses)
	 */
	mrpt::aligned_std_vector<mrpt::poses::CPose3D> left_cam_poses;
	/** true if a checkerboard was correctly detected in both left/right images.
	 * false if it wasn't, so the image pair didn't make it to the optimization.
	 */
	std::vector<bool> image_pair_was_used;

	/** Final reprojection square Root Mean Square Error (in pixels). */
	double final_rmse{0};
	/** Final number of optimization iterations executed. */
	size_t final_iters{0};
	/** Number of image pairs in which valid checkerboards were correctly
	 * detected. */
	size_t final_number_good_image_pairs{0};

	/** The inverse variance (information/precision) of each of the 9 left/right
	 * camera parameters [fx fy cx cy k1 k2 k3 t1 t2].
	 *  Those not estimated as indicated in TStereoCalibParams will be zeros
	 * (i.e. an "infinite uncertainty")
	 */
	Eigen::Array<double, 9, 1> left_params_inv_variance,
		right_params_inv_variance;
};

/**  A list of images, used in checkerBoardStereoCalibration
 * \sa checkerBoardStereoCalibration
 */
using TCalibrationStereoImageList = std::vector<TImageStereoCalibData>;

/** Optimize the calibration parameters of a stereo camera or a RGB+D (Kinect)
 * camera.
 *  This computes the projection and distortion parameters of each camera, and
 * their relative spatial pose,
 *  from a sequence of pairs of captured images of a checkerboard.
 *  A custom implementation of an optimizer (Levenberg-Marquartd) seeks for the
 * set of selected parameters to estimate that minimize the reprojection errors.
 *
 *  \param input_images [IN/OUT] At input, this list must have one entry for
 * each image to process. At output the original, detected checkboard and
 * rectified images can be found here. See TImageCalibData.
 *  \param params [IN] Mandatory: the user must provide the size of the
 * checkerboard, which parameters to optimize and which to leave fixed to zero,
 * etc.
 *  \param out_results [OUT] The results of the calibration, and its
 * uncertainty measure, will be found here upon return.
 *
 * \return false on any error (more info will be dumped to cout), or true on
 * success.
 * \note See also the ready-to-use application: <a
 * href="http://www.mrpt.org/list-of-mrpt-apps/application-kinect-calibrate"
 * >kinect-calibrate</a>
 * \sa CImage::findChessboardCorners, checkerBoardCameraCalibration,
 * mrpt::hwdrivers::CKinect
 */
bool checkerBoardStereoCalibration(
	TCalibrationStereoImageList& images, const TStereoCalibParams& params,
	TStereoCalibResults& out_results);

/** @}  */  // end of grouping
}  // namespace mrpt::vision
