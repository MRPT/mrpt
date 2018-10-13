/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/serialization/CSerializable.h>
#include <mrpt/img/CImage.h>
#include <mrpt/img/TStereoCamera.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/poses/CPose2D.h>

namespace mrpt::obs
{
/** Observation class for either a pair of left+right or left+disparity images
 *from a stereo camera.
 *
 *  To find whether the observation contains a right image and/or a disparity
 *image, see the fields hasImageDisparity and hasImageRight, respectively.
 *   This figure illustrates the coordinate frames involved in this class:
 *
 *	 <center>
 *   <img src="CObservationStereoImages_figRefSystem.png">
 *  </center>
 *
 * \note The images stored in this class can be raw or undistorted images. In
 *the latter case, the "distortion" params of the corresponding "leftCamera" and
 *"rightCamera" fields should be all zeros.
 * \sa CObservation
 * \ingroup mrpt_obs_grp
 */
class CObservationStereoImages : public mrpt::obs::CObservation
{
	DEFINE_SERIALIZABLE(CObservationStereoImages)
	// This must be added for declaration of MEX-related functions
	DECLARE_MEX_CONVERSION

   public:
	CObservationStereoImages() = default;

#ifdef MRPT_HAS_OPENCV
	/** Constructor from "IplImage*" images, which could be NULL.
	 *  The fields hasImageDisparity and hasImageRight will be set to
	 * true/false depending on them being !=nullptr.
	 * Note that the IplImage's will be COPIED, so it's still the caller's
	 * reponsibility to free the original images,
	 *  unless ownMemory is set to true: in that case the IplImage pointers are
	 * copied and those IplImage's will be automatically freed by this object.
	 *
	 */
	CObservationStereoImages(
		IplImage* iplImageLeft, IplImage* iplImageRight,
		IplImage* iplImageDisparity = nullptr, bool ownMemory = false);
#endif

	/** @name Main observation data members
		@{ */

	/** Image from the left camera (this image will be ALWAYS present) \sa
	 * areImagesRectified() */
	mrpt::img::CImage imageLeft;

	/** Image from the right camera, only contains a valid image if
	 * hasImageRight == true. \sa areImagesRectified() */
	mrpt::img::CImage imageRight;

	/** Disparity image, only contains a valid image if hasImageDisparity ==
	 * true.
	 *  The relation between the actual disparity and pixels and each value in
	 * this image is... ???????????  */
	mrpt::img::CImage imageDisparity;

	/** Whether imageDisparity actually contains data (Default upon
	 * construction: false) */
	bool hasImageDisparity{false};
	/** Whether imageRight actually contains data  (Default upon construction:
	 * true) */
	bool hasImageRight{false};

	/** Parameters for the left/right cameras: individual intrinsic and
	 * distortion parameters of the cameras.
	 * See the <a href="http://www.mrpt.org/Camera_Parameters" >tutorial</a>
	 * for a discussion of these parameters.
	 * \sa areImagesRectified(), getStereoCameraParams()
	 */
	mrpt::img::TCamera leftCamera, rightCamera;

	/** The pose of the LEFT camera, relative to the robot. */
	mrpt::poses::CPose3DQuat cameraPose;

	/** The pose of the right camera, relative to the left one:
	 *  Note that using the conventional reference coordinates for the left
	 *   camera (x points to the right, y down), the "right" camera is situated
	 *   at position (BL, 0, 0) with yaw=pitch=roll=0, where BL is the
	 * BASELINE.
	 */
	mrpt::poses::CPose3DQuat rightCameraPose;

	/** Populates a TStereoCamera structure with the parameters in \a
	 * leftCamera, \a rightCamera and \a rightCameraPose \sa
	 * areImagesRectified() */
	void getStereoCameraParams(mrpt::img::TStereoCamera& out_params) const;

	/** Sets \a leftCamera, \a rightCamera and \a rightCameraPose from a
	 * TStereoCamera structure */
	void setStereoCameraParams(const mrpt::img::TStereoCamera& in_params);

	/** This method only checks whether ALL the distortion parameters in \a
	 * leftCamera are set to zero, which is
	 * the convention in MRPT to denote that this pair of stereo images has
	 * been rectified.
	 */
	bool areImagesRectified() const;

	/** @} */

	// See base class docs
	void getSensorPose(mrpt::poses::CPose3D& out_sensorPose) const override
	{
		out_sensorPose = mrpt::poses::CPose3D(cameraPose);
	}
	void setSensorPose(const mrpt::poses::CPose3D& newSensorPose) override
	{
		cameraPose = mrpt::poses::CPose3DQuat(newSensorPose);
	}
	void getDescriptionAsText(std::ostream& o) const override;

	/** Do an efficient swap of all data members of this object with "o". */
	void swap(CObservationStereoImages& o);

};  // End of class def.

}  // namespace mrpt::obs
// Add for declaration of mexplus::from template specialization
DECLARE_MEXPLUS_FROM(mrpt::obs::CObservationStereoImages)
