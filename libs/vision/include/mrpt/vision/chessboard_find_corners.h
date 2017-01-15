/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef mrpt_vision_find_chessboard_H
#define mrpt_vision_find_chessboard_H

#include <mrpt/utils/CImage.h>

#include <mrpt/vision/types.h>
#include <mrpt/vision/link_pragmas.h>

namespace mrpt
{
	namespace vision
	{
		/** \addtogroup chessboard_calib
		    @{ */

		/** Look for the corners of a chessboard in the image using one of two different methods.
		  *
		  *  The search algorithm will be OpenCV's function cvFindChessboardCorners or its improved
		  *   version published by M. Rufli, D. Scaramuzza, and R. Siegwart. See: http://robotics.ethz.ch/~scaramuzza/Davide_Scaramuzza_files/Research/OcamCalib_Tutorial.htm
		  *    and the papers:
		  *		- 1. Scaramuzza, D., Martinelli, A. and Siegwart, R. (2006), A Toolbox for Easily Calibrating Omnidirectional Cameras, Proceedings of the IEEE/RSJ International Conference on Intelligent Robots and Systems  (IROS 2006), Beijing, China, October 2006.
		  *		- 2. Scaramuzza, D., Martinelli, A. and Siegwart, R., (2006). "A Flexible Technique for Accurate Omnidirectional Camera Calibration and Structure from Motion", Proceedings of IEEE International Conference of Vision Systems  (ICVS'06), New York, January 5-7, 2006.
		  *		- 3. Rufli, M., Scaramuzza, D., and Siegwart, R. (2008), Automatic Detection of Checkerboards on Blurred and Distorted Images, Proceedings of the IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS 2008), Nice, France, September 2008.
		  *
		  *  After detecting the corners with either method, it's called "cvFindCornerSubPix" to achieve subpixel accuracy.
		  *
		  * \param cornerCoords [OUT] The pixel coordinates of all the corners.
		  * \param check_size_x [IN] The number of squares, in the X direction
		  * \param check_size_y [IN] The number of squares, in the Y direction
		  * \param normalize_image [IN] Whether to normalize the image before detection
		  * \param useScaramuzzaMethod [IN] Whether to use the alternative, more robust method by M. Rufli, D. Scaramuzza, and R. Siegwart.
		  *
		  * \return true on success
		  *
		  * \sa findMultipleChessboardsCorners, mrpt::vision::checkerBoardCameraCalibration, drawChessboardCorners
		  */
		bool VISION_IMPEXP findChessboardCorners(
			const mrpt::utils::CImage &img,
			std::vector<mrpt::utils::TPixelCoordf> 	&cornerCoords,
			unsigned int  check_size_x,
			unsigned int  check_size_y,
			bool  normalize_image = true,
			bool  useScaramuzzaMethod = false );

		/** Look for the corners of one or more chessboard/checkerboards in the image.
		  *  This method uses an improved version of OpenCV's cvFindChessboardCorners published
		  *   by M. Rufli, D. Scaramuzza, and R. Siegwart. See: http://robotics.ethz.ch/~scaramuzza/Davide_Scaramuzza_files/Research/OcamCalib_Tutorial.htm
		  *    and the papers:
		  *		- 1. Scaramuzza, D., Martinelli, A. and Siegwart, R. (2006), A Toolbox for Easily Calibrating Omnidirectional Cameras, Proceedings of the IEEE/RSJ International Conference on Intelligent Robots and Systems  (IROS 2006), Beijing, China, October 2006.
		  *		- 2. Scaramuzza, D., Martinelli, A. and Siegwart, R., (2006). "A Flexible Technique for Accurate Omnidirectional Camera Calibration and Structure from Motion", Proceedings of IEEE International Conference of Vision Systems  (ICVS'06), New York, January 5-7, 2006.
		  *		- 3. Rufli, M., Scaramuzza, D., and Siegwart, R. (2008), Automatic Detection of Checkerboards on Blurred and Distorted Images, Proceedings of the IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS 2008), Nice, France, September 2008.
		  *
		  *  That method has been extended in this MRPT implementation to automatically detect a
		  *   number of different checkerboards in the same image.
		  *
		  * \param cornerCoords [OUT] A vector of N vectors of pixel coordinates, for each of the N chessboards detected.
		  * \param check_size_x [IN] The number of squares, in the X direction
		  * \param check_size_y [IN] The number of squares, in the Y direction
		  *
		  *
		  * \sa mrpt::vision::checkerBoardCameraCalibration, drawChessboardCorners
		  */
		void VISION_IMPEXP findMultipleChessboardsCorners(
			const mrpt::utils::CImage &img,
			std::vector<std::vector<mrpt::utils::TPixelCoordf> > 	&cornerCoords,
			unsigned int  check_size_x,
			unsigned int  check_size_y );

		/** @} */
	}
}
#endif

