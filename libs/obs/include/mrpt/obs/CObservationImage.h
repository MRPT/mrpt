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
#include <mrpt/img/TCamera.h>
#include <mrpt/math/CMatrix.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose2D.h>

namespace mrpt::obs
{
/** Declares a class derived from "CObservation" that encapsules an image from a
 camera, whose relative pose to robot is also stored.
	 The next figure illustrate the coordinates reference systems involved in
 this class:<br>
	 <center>
	 <img src="CObservationImage_figRefSystem.png">
	 </center>
 *
 * \sa CObservation, CObservationStereoImages
 * \ingroup mrpt_obs_grp
 */
class CObservationImage : public CObservation
{
	DEFINE_SERIALIZABLE(CObservationImage)
	// This must be added for declaration of MEX-related functions
	DECLARE_MEX_CONVERSION

   public:
	/** Constructor.
	 * \param iplImage An OpenCV "IplImage*" object with the image to be loaded
	 * in the member "image", or nullptr (default) for an empty image.
	 *
	 */
	CObservationImage() = default;
#ifdef MRPT_HAS_OPENCV
	CObservationImage(const IplImage* ipl);
#endif
	/** The pose of the camera on the robot
	 */
	mrpt::poses::CPose3D cameraPose;

	/** Intrinsic and distortion parameters of the camera.
	 * See the <a href="http://www.mrpt.org/Camera_Parameters" >tutorial</a>
	 * for a discussion of these parameters.
	 */
	mrpt::img::TCamera cameraParams;

	/** The image captured by the camera, that is, the main piece of information
	 * of this observation. */
	mrpt::img::CImage image;

	/** Computes the rectified (un-distorted) image, using the embeded
	 * distortion parameters.
	 */
	void getRectifiedImage(mrpt::img::CImage& out_img) const;

	// See base class docs
	void getSensorPose(mrpt::poses::CPose3D& out_sensorPose) const override
	{
		out_sensorPose = cameraPose;
	}
	void setSensorPose(const mrpt::poses::CPose3D& newSensorPose) override
	{
		cameraPose = newSensorPose;
	}
	void getDescriptionAsText(std::ostream& o) const override;

};  // End of class def.

}  // namespace mrpt::obs
// Add for declaration of mexplus::from template specialization
DECLARE_MEXPLUS_FROM(mrpt::obs::CObservationImage)
