/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
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
		/** Use this class to undistort monocular images if the same distortion map is used over and over again.
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
		  * \sa CStereoRectifyMap, mrpt::utils::TCamera, the application <a href="http://www.mrpt.org/Application:camera-calib" >camera-calib</a> for calibrating a camera.
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
