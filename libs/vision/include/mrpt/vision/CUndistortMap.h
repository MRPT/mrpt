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
#ifndef mrpt_CUndistortMap_H
#define mrpt_CUndistortMap_H

#include <mrpt/utils/TCamera.h>
#include <mrpt/utils/CImage.h>

#include <mrpt/vision/link_pragmas.h>

namespace mrpt
{
	namespace vision
	{
		/** Use this class to undistort images if the same distortion map is used over and over again.
		  *  Using this class is much more efficient that calling mrpt::utils::CImage::rectifyImage or OpenCV's cvUndistort2(), since
		  *  the remapping data is computed only once for the camera parameters (typical times: 640x480 image -> 70% build map / 30% actual undistort).
		  *
		  *  Works with grayscale or color images.
		  *
		  * Example of usage:
		  * \code
		  *   CUndistortMap   unmap;
		  *   mrpt::utils::TCamera  cam_params;
		  *
		  *   unmap.setFromCamParams( cam_params );
		  *
		  *   mrpt::utils::CImage  img, img_out;
		  *
		  *   while (true) {
		  *     unmap.undistort(img, img_out);  // or:
		  *     unmap.undistort(img);  // output in place
		  *   }
		  *
		  * \endcode
		  *
		  * \sa mrpt::utils::TCamera, the application <a href="http://www.mrpt.org/Application:camera-calib" >camera-calib</a> for calibrating a camera.
		  * \ingroup mrpt_vision_grp
		  */
		class VISION_IMPEXP  CUndistortMap
		{
		public:
			CUndistortMap(); //!< Default ctor

			/** Prepares the mapping from the distortion parameters of a camera.
			  * Must be called before invoking \a undistort().
			  */
			void setFromCamParams(const mrpt::utils::TCamera &params);

			/** Undistort the input image and saves the result in the output one - \a setFromCamParams() must have been set prior to calling this.
			  */
			void undistort(const mrpt::utils::CImage &in_img, mrpt::utils::CImage &out_img) const;

			/** Undistort the input image and saves the result in-place- \a setFromCamParams() must have been set prior to calling this.
			  */
			void undistort(mrpt::utils::CImage &in_out_img) const;

			/** Returns the camera parameters which were used to generate the distortion map, as passed by the user to \a setFromCamParams */
			inline const mrpt::utils::TCamera & getCameraParams() const { return m_camera_params; }

			/** Returns true if \a setFromCamParams() has been already called, false otherwise.
			  *  Can be used within loops to determine the first usage of the object and when it needs to be initialized.
			  */
			inline bool isSet() const { return !m_dat_mapx.empty(); }

		private:
			std::vector<int16_t>  m_dat_mapx;
			std::vector<uint16_t> m_dat_mapy;

			mrpt::utils::TCamera  m_camera_params; //!< A copy of the data provided by the user

		}; // end class
	} // end namespace
} // end namespace
#endif
