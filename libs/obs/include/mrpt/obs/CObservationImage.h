/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/img/CImage.h>
#include <mrpt/img/TCamera.h>
#include <mrpt/math/CMatrixF.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/serialization/CSerializable.h>

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
	DEFINE_SERIALIZABLE(CObservationImage, mrpt::obs)
	// This must be added for declaration of MEX-related functions
	DECLARE_MEX_CONVERSION

   public:
	/** Constructor.
	 * \param iplImage An OpenCV "IplImage*" object with the image to be loaded
	 * in the member "image", or nullptr (default) for an empty image.
	 *
	 */
	CObservationImage() = default;
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

	/** Computes the un-distorted image, using the embeded camera
	 * intrinsic & distortion parameters.
	 */
	void getUndistortedImage(mrpt::img::CImage& out_img) const;

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

	/** @name Delayed-load (lazy-load) manual control methods.
		@{ */

	/** Makes sure the image, which may be externally stored, are loaded in
	 * memory. \sa unload
	 */
	void load_impl() const override;

	/** Unload image, for the case of it being stored in lazy-load mode
	 *  (othewise, the method has no effect).
	 * \sa load
	 */
	void unload() const override;
	/** @} */

};	// End of class def.

}  // namespace mrpt::obs
// Add for declaration of mexplus::from template specialization
DECLARE_MEXPLUS_FROM(mrpt::obs::CObservationImage)
