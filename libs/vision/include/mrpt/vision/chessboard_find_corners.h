/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
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
			std::vector<TPixelCoordf> 	&cornerCoords,
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
			std::vector<std::vector<TPixelCoordf> > 	&cornerCoords,
			unsigned int  check_size_x,
			unsigned int  check_size_y );

		/** @} */
	}
}
#endif

