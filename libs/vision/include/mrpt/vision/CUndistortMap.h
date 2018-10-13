/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/img/TCamera.h>
#include <mrpt/img/CImage.h>

namespace mrpt::vision
{
/** Use this class to undistort monocular images if the same distortion map is
 * used over and over again.
 *  Using this class is much more efficient that calling
 * mrpt::img::CImage::rectifyImage or OpenCV's cvUndistort2(), since
 *  the remapping data is computed only once for the camera parameters (typical
 * times: 640x480 image -> 70% build map / 30% actual undistort).
 *
 *  Works with grayscale or color images.
 *
 * Example of usage:
 * \code
 *   CUndistortMap   unmap;
 *   mrpt::img::TCamera  cam_params;
 *
 *   unmap.setFromCamParams( cam_params );
 *
 *   mrpt::img::CImage  img, img_out;
 *
 *   while (true) {
 *     unmap.undistort(img, img_out);  // or:
 *     unmap.undistort(img);  // output in place
 *   }
 *
 * \endcode
 *
 * \sa CStereoRectifyMap, mrpt::img::TCamera, the application <a
 * href="http://www.mrpt.org/Application:camera-calib" >camera-calib</a> for
 * calibrating a camera.
 * \ingroup mrpt_vision_grp
 */
class CUndistortMap
{
   public:
	/** Default ctor */
	CUndistortMap();

	/** Prepares the mapping from the distortion parameters of a camera.
	 * Must be called before invoking \a undistort().
	 */
	void setFromCamParams(const mrpt::img::TCamera& params);

	/** Undistort the input image and saves the result in the output one - \a
	 * setFromCamParams() must have been set prior to calling this.
	 */
	void undistort(
		const mrpt::img::CImage& in_img, mrpt::img::CImage& out_img) const;

	/** Undistort the input image and saves the result in-place- \a
	 * setFromCamParams() must have been set prior to calling this.
	 */
	void undistort(mrpt::img::CImage& in_out_img) const;

	/** Returns the camera parameters which were used to generate the distortion
	 * map, as passed by the user to \a setFromCamParams */
	inline const mrpt::img::TCamera& getCameraParams() const
	{
		return m_camera_params;
	}

	/** Returns true if \a setFromCamParams() has been already called, false
	 * otherwise.
	 *  Can be used within loops to determine the first usage of the object and
	 * when it needs to be initialized.
	 */
	inline bool isSet() const { return !m_dat_mapx.empty(); }

   private:
	std::vector<int16_t> m_dat_mapx;
	std::vector<uint16_t> m_dat_mapy;

	/** A copy of the data provided by the user */
	mrpt::img::TCamera m_camera_params;

};  // end class
}  // namespace mrpt::vision
