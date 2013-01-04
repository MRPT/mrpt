/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
   +---------------------------------------------------------------------------+ */

#ifndef mrpt_vision_pinhole_H
#define mrpt_vision_pinhole_H

#include <mrpt/utils/TCamera.h>
#include <mrpt/vision/utils.h>

namespace mrpt
{
	namespace vision
	{
		/** Functions related to pinhole camera models, point projections, etc. \ingroup mrpt_vision_grp */
		namespace pinhole
		{
			/** \addtogroup mrpt_vision_grp 
			  * @{ */
		
			using mrpt::utils::TPixelCoordf;

			/** Project a set of 3D points into a camera at an arbitrary 6D pose using its calibration matrix (undistorted projection model)
			  * \param in_points_3D [IN] The list of 3D points in world coordinates (meters) to project.
			  * \param cameraPose [IN] The pose of the camera in the world.
			  * \param intrinsicParams [IN] The 3x3 calibration matrix. See http://www.mrpt.org/Camera_Parameters
			  * \param projectedPoints [OUT] The list of image coordinates (in pixels) for the projected points. At output this list is resized to the same number of input points.
			  * \param accept_points_behind [IN] See the note below.
			  *
			  * \note Points "behind" the camera (which couldn't be physically seen in the real world) are marked with pixel coordinates (-1,-1) to detect them as invalid, unless accept_points_behind is true. In that case they'll be projected normally.
			  *
			  * \sa projectPoints_with_distortion, projectPoint_no_distortion
			  */
			void VISION_IMPEXP projectPoints_no_distortion(
				const std::vector<mrpt::poses::CPoint3D> &in_points_3D,
				const mrpt::poses::CPose3D &cameraPose,
				const mrpt::math::CMatrixDouble33 & intrinsicParams,
				std::vector<TPixelCoordf> &projectedPoints,
				bool accept_points_behind = false
				);

			/** Project a single 3D point with global coordinates P into a camera at pose F, without distortion parameters.
			  *  The template argument INVERSE_CAM_POSE is related on how the camera pose "F" is stored:
			  *		- INVERSE_CAM_POSE:false -> The local coordinates of the feature wrt the camera F are: \f$ P \ominus F \f$
			  *		- INVERSE_CAM_POSE:true  -> The local coordinates of the feature wrt the camera F are: \f$ F \oplus P \f$
			  */
			template <bool INVERSE_CAM_POSE>
			inline TPixelCoordf projectPoint_no_distortion(
				const mrpt::utils::TCamera  &cam_params,
				const mrpt::poses::CPose3D  &F,
				const mrpt::math::TPoint3D &P)
			{
				double x,y,z; // wrt cam (local coords)
				if (INVERSE_CAM_POSE)
					F.composePoint(P.x,P.y,P.z,  x,y,z);
				else
					F.inverseComposePoint(P.x,P.y,P.z, x,y,z);
				ASSERT_(z!=0)
				// Pinhole model:
				return TPixelCoordf(
					cam_params.cx() + cam_params.fx() * x/z,
					cam_params.cy() + cam_params.fy() * y/z );
			}

			//! \overload 
			template <typename POINT>
			inline void projectPoint_no_distortion(
				const POINT  &in_point_wrt_cam,
				const mrpt::utils::TCamera  &cam_params,
				TPixelCoordf  &out_projectedPoints )
			{
				ASSERT_(in_point_wrt_cam.z!=0)
				// Pinhole model:
				out_projectedPoints.x = cam_params.cx() + cam_params.fx() * in_point_wrt_cam.x/in_point_wrt_cam.z;
				out_projectedPoints.y = cam_params.cy() + cam_params.fy() * in_point_wrt_cam.y/in_point_wrt_cam.z;
			}


			/** Project a set of 3D points into a camera at an arbitrary 6D pose using its calibration matrix and distortion parameters (radial and tangential distortions projection model)
			  * \param in_points_3D [IN] The list of 3D points in world coordinates (meters) to project.
			  * \param cameraPose [IN] The pose of the camera in the world.
			  * \param intrinsicParams [IN] The 3x3 calibration matrix. See http://www.mrpt.org/Camera_Parameters
			  * \param distortionParams [IN] The 4-length vector with the distortion parameters [k1 k2 p1 p2]. See http://www.mrpt.org/Camera_Parameters
			  * \param projectedPoints [OUT] The list of image coordinates (in pixels) for the projected points. At output this list is resized to the same number of input points.
			  * \param accept_points_behind [IN] See the note below.
			  *
			  * \note Points "behind" the camera (which couldn't be physically seen in the real world) are marked with pixel coordinates (-1,-1) to detect them as invalid, unless accept_points_behind is true. In that case they'll be projected normally.
			  *
			  * \sa projectPoint_with_distortion, projectPoints_no_distortion
			  */
			void VISION_IMPEXP projectPoints_with_distortion(
				const std::vector<mrpt::poses::CPoint3D> &in_points_3D,
				const mrpt::poses::CPose3D &cameraPose,
				const mrpt::math::CMatrixDouble33 & intrinsicParams,
				const std::vector<double> & distortionParams,
				std::vector<TPixelCoordf> &projectedPoints,
				bool accept_points_behind = false
				);

			/** Project one 3D point into a camera using its calibration matrix and distortion parameters (radial and tangential distortions projection model)
			  * \param in_point_wrt_cam [IN] The 3D point wrt the camera focus, with +Z=optical axis, +X=righthand in the image plane, +Y=downward in the image plane.
			  * \param in_cam_params [IN] The camera parameters. See http://www.mrpt.org/Camera_Parameters
			  * \param out_projectedPoints [OUT] The projected point, in pixel units.
			  * \param accept_points_behind [IN] See the note below.
			  *
			  * \note Points "behind" the camera (which couldn't be physically seen in the real world) are marked with pixel coordinates (-1,-1) to detect them as invalid, unless accept_points_behind is true. In that case they'll be projected normally.
			  *
			  * \sa projectPoints_with_distortion
			  */
			void VISION_IMPEXP projectPoint_with_distortion(
				const mrpt::math::TPoint3D  &in_point_wrt_cam,
				const mrpt::utils::TCamera  &in_cam_params,
				TPixelCoordf  &out_projectedPoints,
				bool accept_points_behind = false
				);

			//! \overload
			void VISION_IMPEXP projectPoints_with_distortion(
				const std::vector<mrpt::math::TPoint3D>  &P,
				const mrpt::utils::TCamera  &params,
				const CPose3DQuat &cameraPose,
				std::vector<TPixelCoordf>  &pixels,
				bool accept_points_behind = false
				);


			/** Undistort a list of points given by their pixel coordinates, provided the camera matrix and distortion coefficients.
			  * \param srcDistortedPixels [IN] The pixel coordinates as in the distorted image.
			  * \param dstUndistortedPixels [OUT] The computed pixel coordinates without distortion.
			  * \param intrinsicParams [IN] The 3x3 calibration matrix. See http://www.mrpt.org/Camera_Parameters
			  * \param distortionParams [IN] The 4-length vector with the distortion parameters [k1 k2 p1 p2]. See http://www.mrpt.org/Camera_Parameters
			  * \sa undistort_point
			  */
			void VISION_IMPEXP undistort_points(
				const std::vector<TPixelCoordf>  &srcDistortedPixels,
				std::vector<TPixelCoordf> &dstUndistortedPixels,
				const mrpt::math::CMatrixDouble33 & intrinsicParams,
				const std::vector<double> & distortionParams );

			/** Undistort a list of points given by their pixel coordinates, provided the camera matrix and distortion coefficients.
			  * \param srcDistortedPixels [IN] The pixel coordinates as in the distorted image.
			  * \param dstUndistortedPixels [OUT] The computed pixel coordinates without distortion.
			  * \param cameraModel [IN] The camera parameters.
			  * \sa undistort_point
			  */
			void VISION_IMPEXP undistort_points(
				const std::vector<TPixelCoordf>  &srcDistortedPixels,
				std::vector<TPixelCoordf> &dstUndistortedPixels,
				const mrpt::utils::TCamera  &cameraModel);

			/** Undistort one point given by its pixel coordinates and the camera parameters.
			  * \sa undistort_points
			  */
			void VISION_IMPEXP undistort_point(
				const TPixelCoordf  &inPt,
				TPixelCoordf        &outPt,
				const mrpt::utils::TCamera  &cameraModel);

			/** @} */ // end of grouping
		}
	}
}

#endif
