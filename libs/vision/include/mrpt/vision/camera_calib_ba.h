/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2010  University of Malaga                           |
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

#ifndef mrpt_vision_camera_calib_ba_H
#define mrpt_vision_camera_calib_ba_H

#include <mrpt/utils/types.h>
#include <mrpt/utils/TCamera.h>

#include <mrpt/vision/link_pragmas.h>

namespace mrpt
{
	namespace vision
	{
		/** Data returned by  mrpt::vision::camera_calib_ba */
		struct VISION_IMPEXP TCamCalibBAResults
		{
			std::vector<mrpt::poses::CPose3DQuat>  	camera_poses;
			std::vector<mrpt::math::TPoint3D>		landmark_positions;
		};

		/** Optimizes the camera intrinsic and distortion parameters given a sequence of features tracked along a few consecutive frames.
		  *  This method allows calibrating a camera without a checkerboard or any other special pattern, just by moving the camera and tracking
		  *  random features.
		  *
		  * \return Returns the average pixel reprojection error with the final estimated parameters.
		  *
		  */
		double VISION_IMPEXP camera_calib_ba(
			const std::vector< std::vector<mrpt::utils::TPixelCoordf> >  & in_tracked_feats,
			unsigned int camera_ncols,
			unsigned int camera_nrows,
			mrpt::utils::TCamera &out_optimal_params,
			TCamCalibBAResults &out_info
			);
	}
}

#endif

