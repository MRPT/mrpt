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
#include <mrpt/obs/CObservation.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/poses/CPose2D.h>

namespace mrpt::obs
{
struct TStereoImageFeatures
{
	std::pair<mrpt::img::TPixelCoordf, mrpt::img::TPixelCoordf> pixels;
	unsigned int ID;
};

/** Declares a class derived from "CObservation" that encapsules a pair of
 cameras and a set of matched image features extracted from them.
 *
 <b>NOTE:</b> The image features stored in this class are NOT supposed to be
 UNDISTORTED, but the TCamera members must provide their distortion params.
 A zero-vector of distortion params means a set of UNDISTORTED pixels.<br>
 * \sa CObservation
 * \ingroup mrpt_obs_grp
 */
class CObservationStereoImagesFeatures : public CObservation
{
	DEFINE_SERIALIZABLE(CObservationStereoImagesFeatures)

   public:
	CObservationStereoImagesFeatures() = default;
	/** Other constructor providing members initialization.
	 */
	CObservationStereoImagesFeatures(
		const mrpt::img::TCamera& cLeft /*left camera*/,
		const mrpt::img::TCamera& cRight /*right camera*/,
		const mrpt::poses::CPose3DQuat& rCPose /*rightCameraPose*/,
		const mrpt::poses::CPose3DQuat& cPORobot /*cameraPoseOnRobot*/);

	/** A method for storing the set of observed features in a text file in the
	 * format: <br>
	 * ID ul vl ur vr <br>
	 * being (ul,vl) and (ur,vr) the "x" and "y" coordinates for the left and
	 * right feature, respectively.
	 */
	void saveFeaturesToTextFile(const std::string& filename);

	// ------------------
	// Class Members
	// ------------------
	mrpt::img::TCamera cameraLeft, cameraRight;

	/** The pose of the right camera, relative to the left one:
	 *  Note that for the Bumblebee stereo camera and using the conventional
	 * reference coordinates for the left
	 *   camera ("x" points to the right, "y" down), the "right" camera is
	 * situated
	 *   at position (BL, 0, 0) with q = [1 0 0 0], where BL is the BASELINE.
	 */
	mrpt::poses::CPose3DQuat rightCameraPose;

	/** The pose of the LEFT camera, relative to the robot.
	 */
	mrpt::poses::CPose3DQuat cameraPoseOnRobot;

	/** Vectors of image feature pairs (with ID).
	 */
	std::vector<TStereoImageFeatures> theFeatures;

	// See base class docs
	void getSensorPose(mrpt::poses::CPose3D& out_sensorPose) const override
	{
		out_sensorPose = mrpt::poses::CPose3D(cameraPoseOnRobot);
	}
	// See base class docs
	void getSensorPose(mrpt::poses::CPose3DQuat& out_sensorPose) const
	{
		out_sensorPose = cameraPoseOnRobot;
	}
	// See base class docs
	void getDescriptionAsText(std::ostream& o) const override;

	/** A general method to change the sensor pose on the robot in a
	 * mrpt::poses::CPose3D form.
	 *  Note that most sensors will use the full (6D) CPose3DQuat, but see the
	 * derived classes for more details or special cases.
	 * \sa getSensorPose
	 */
	inline void setSensorPose(
		const mrpt::poses::CPose3D& newSensorPose) override
	{
		cameraPoseOnRobot = mrpt::poses::CPose3DQuat(newSensorPose);
	}

	/** A general method to change the sensor pose on the robot in a CPose3DQuat
	 * form.
	 *  Note that most sensors will use the full (6D) CPose3DQuat, but see the
	 * derived classes for more details or special cases.
	 * \sa getSensorPose
	 */
	inline void setSensorPose(const mrpt::poses::CPose3DQuat& newSensorPose)
	{
		cameraPoseOnRobot = newSensorPose;
	}
};  // End of class def.

}  // namespace mrpt::obs
