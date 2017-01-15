/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef mrpt_vision_chessboard_camera_calib_H
#define mrpt_vision_chessboard_camera_calib_H

#include <mrpt/utils/CImage.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/math/CMatrixFixedNumeric.h>
#include <mrpt/vision/types.h>
#include <mrpt/vision/link_pragmas.h>

namespace mrpt
{
	namespace vision
	{
		/** \addtogroup chessboard_calib Chessboard calibration
		  *  \ingroup mrpt_vision_grp
		  *  @{  */

		/** Data associated to each image in the calibration process mrpt::vision::checkerBoardCameraCalibration (All the information can be left empty and will be filled up in the calibration method).
		  */
		struct VISION_IMPEXP TImageCalibData
		{
			mrpt::utils::CImage	img_original;     //!< This image will be automatically loaded from the file name passed to checkerBoardCameraCalibration
			mrpt::utils::CImage	img_checkboard;   //!< At output, this will contain the detected checkerboard overprinted to the image.
			mrpt::utils::CImage	img_rectified;    //!< At output, this will be the rectified image
			std::vector<mrpt::utils::TPixelCoordf>	detected_corners; //!< At output, the detected corners (x,y) in pixel units.
			mrpt::poses::CPose3D			reconstructed_camera_pose;   //!< At output, the reconstructed pose of the camera.
			std::vector<mrpt::utils::TPixelCoordf>		projectedPoints_distorted;   //!< At output, only will have an empty vector if the checkerboard was not found in this image, or the predicted (reprojected) corners, which were used to estimate the average square error.
			std::vector<mrpt::utils::TPixelCoordf>		projectedPoints_undistorted; //!< At output, like projectedPoints_distorted but for the undistorted image.

			/** Empty all the data */
			void clear() { *this = TImageCalibData(); }
		};

		/**  A list of images, used in checkerBoardCameraCalibration
		  * \sa checkerBoardCameraCalibration
		  */
		typedef std::map<std::string,TImageCalibData> TCalibrationImageList;

		/** Performs a camera calibration (computation of projection and distortion parameters) from a sequence of captured images of a checkerboard.
		  * \param input_images [IN/OUT] At input, this list must have one entry for each image to process. At output the original, detected checkboard and rectified images can be found here. See TImageCalibData.
		  * \param check_size_x [IN] The number of squares in the checkerboard in the X direction.
		  * \param check_size_y [IN] The number of squares in the checkerboard in the Y direction.
		  * \param check_squares_length_X_meters [IN] The size of each square in the checkerboard, in meters, in the X axis.
		  * \param check_squares_length_Y_meters [IN] This will typically be equal to check_squares_length_X_meters.
		  * \param intrinsicParams [OUT] The 3x3 intrinsic parameters matrix. See http://www.mrpt.org/Camera_Parameters
		  * \param distortionParams [OUT] The 1x4 vector of distortion parameters: k1 k2 p1 p2. See http://www.mrpt.org/Camera_Parameters
		  * \param normalize_image [IN] Select OpenCV flag
		  * \param out_MSE  [OUT] If set to !=NULL, the mean square error of the reprojection will be stored here (in pixel units).
		  * \param skipDrawDetectedImgs [IN] Whether to skip the generation of the undistorted and detected images in each TImageCalibData
		  * \param useScaramuzzaAlternativeDetector [IN] Whether to use an alternative detector. See CImage::findChessboardCorners for more deatails and references.
		  * \sa The <a href="http://www.mrpt.org/Application:camera-calib-gui" >camera-calib-gui application</a> is a user-friendly GUI to this class.
		  * \return false on any error (more info will be dumped to cout), or true on success.
		  * \sa CImage::findChessboardCorners, checkerBoardStereoCalibration
		  */
		bool VISION_IMPEXP checkerBoardCameraCalibration(
			TCalibrationImageList &images,
			unsigned int  check_size_x,
			unsigned int  check_size_y,
			double        check_squares_length_X_meters,
			double        check_squares_length_Y_meters,
			mrpt::utils::TCamera   &out_camera_params,
			bool		normalize_image = true,
			double            *out_MSE = NULL,
			bool               skipDrawDetectedImgs = false,
			bool			   useScaramuzzaAlternativeDetector = false
			);

		/** Performs a camera calibration (computation of projection and distortion parameters) from a sequence of captured images of a checkerboard.
		  * \param input_images [IN/OUT] At input, this list must have one entry for each image to process. At output the original, detected checkboard and rectified images can be found here. See TImageCalibData.
		  * \param check_size_x [IN] The number of squares in the checkerboard in the X direction.
		  * \param check_size_y [IN] The number of squares in the checkerboard in the Y direction.
		  * \param check_squares_length_X_meters [IN] The size of each square in the checkerboard, in meters, in the X axis.
		  * \param check_squares_length_Y_meters [IN] This will typically be equal to check_squares_length_X_meters.
		  * \param intrinsicParams [OUT] The 3x3 intrinsic parameters matrix. See http://www.mrpt.org/Camera_Parameters
		  * \param distortionParams [OUT] The 1x4 vector of distortion parameters: k1 k2 p1 p2. See http://www.mrpt.org/Camera_Parameters
		  * \param normalize_image [IN] Select OpenCV flag
		  * \param out_MSE  [OUT] If set to !=NULL, the mean square error of the reprojection will be stored here (in pixel units).
		  * \param skipDrawDetectedImgs [IN] Whether to skip the generation of the undistorted and detected images in each TImageCalibData
		  * \param useScaramuzzaAlternativeDetector [IN] Whether to use an alternative detector. See CImage::findChessboardCorners for more deatails and references.
		  * \sa The <a href="http://www.mrpt.org/Application:camera-calib-gui" >camera-calib-gui application</a> is a user-friendly GUI to this class.
		  * \return false on any error (more info will be dumped to cout), or true on success.
		  * \sa CImage::findChessboardCorners
		  */
		bool VISION_IMPEXP checkerBoardCameraCalibration(
			TCalibrationImageList &images,
			unsigned int  check_size_x,
			unsigned int  check_size_y,
			double        check_squares_length_X_meters,
			double        check_squares_length_Y_meters,
			mrpt::math::CMatrixDouble33			&intrinsicParams,
			std::vector<double>		&distortionParams,
			bool		normalize_image = true,
			double            *out_MSE = NULL,
			bool               skipDrawDetectedImgs = false,
			bool			   useScaramuzzaAlternativeDetector = false
			);

		/** @}  */ // end of grouping

	}
}


#endif
