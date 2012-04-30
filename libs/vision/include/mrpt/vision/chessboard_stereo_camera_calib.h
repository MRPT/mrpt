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

#ifndef mrpt_vision_chessboard_stereo_calib_H
#define mrpt_vision_chessboard_stereo_calib_H

#include <mrpt/utils/CImage.h>
#include <mrpt/utils/TStereoCamera.h>
#include <mrpt/vision/types.h>
#include <mrpt/vision/chessboard_camera_calib.h>
#include <mrpt/vision/link_pragmas.h>

namespace mrpt
{
	namespace vision
	{
		using namespace mrpt::utils;

		/** \addtogroup chessboard_calib Chessboard calibration
		  *  \ingroup mrpt_vision_grp
		  *  @{  */

		/** Data associated to each stereo image in the calibration process mrpt::vision::checkerBoardCameraCalibration (All the information can be left empty and will be filled up in the calibration method).
		  */
		struct VISION_IMPEXP TImageStereoCalibData
		{
			TImageCalibData left, right;
		};

		/** Input parameters for mrpt::vision::checkerBoardStereoCalibration */
		struct VISION_IMPEXP TStereoCalibParams
		{
			unsigned int  check_size_x,check_size_y; //!< The number of squares in the checkerboard in the "X" & "Y" direction.
			double        check_squares_length_X_meters,check_squares_length_Y_meters; //!< The size of each square in the checkerboard, in meters, in the "X" & "Y" axes.
			bool          normalize_image;
			bool          skipDrawDetectedImgs;
			bool          verbose;                   //!< Show progress messages to std::cout console (default=true)


			TStereoCalibParams() : 
				check_size_x(7),check_size_y(9),
				check_squares_length_X_meters(0.02),check_squares_length_Y_meters(0.02),
				normalize_image(true),
				skipDrawDetectedImgs(false),
				verbose(true)
			{}
		};

		/** Output results for mrpt::vision::checkerBoardStereoCalibration */
		struct VISION_IMPEXP TStereoCalibResults
		{
			mrpt::utils::TStereoCamera  cam_params;  //!< Recovered parameters of the stereo camera
			// Uncertainty...
		};

		/**  A list of images, used in checkerBoardStereoCalibration
		  * \sa checkerBoardStereoCalibration
		  */
		typedef std::vector<TImageStereoCalibData> TCalibrationStereoImageList;

		/** Performs stereo camera calibration (computation of projection and distortion parameters) from a sequence of pairs of captured images of a checkerboard.
		  * \param input_images [IN/OUT] At input, this list must have one entry for each image to process. At output the original, detected checkboard and rectified images can be found here. See TImageCalibData.
		  * \return false on any error (more info will be dumped to cout), or true on success.
		  * \sa CImage::findChessboardCorners, checkerBoardCameraCalibration
		  */
		bool VISION_IMPEXP checkerBoardStereoCalibration(
			TCalibrationStereoImageList & images,
			const TStereoCalibParams    & params,
			TStereoCalibResults         & out_results
			);

		/** @}  */ // end of grouping

	}
}


#endif
