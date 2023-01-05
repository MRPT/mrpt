/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/obs/CObservation.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/serialization/CSerializable.h>

#include <deque>

namespace mrpt::obs
{
/** An observation of one or more "features" or "objects", possibly identified
 * with a unique ID, whose relative SE(3) pose is observed with respect to the
 * sensor.
 * The list of features is stored in \a sensedFeatures
 * \sa CObservation
 * \ingroup mrpt_obs_grp
 */
class CObservation6DFeatures : public CObservation
{
	DEFINE_SERIALIZABLE(CObservation6DFeatures, mrpt::obs)
   public:
	/** Default ctor */
	CObservation6DFeatures();

	/** Information about the sensor */
	float minSensorDistance{0}, maxSensorDistance{1e6f};

	/** Each one of the measurements */
	struct TMeasurement
	{
		/** The observed feature SE(3) pose with respect to the sensor */
		mrpt::poses::CPose3D pose;
		/** The feature ID, or INVALID_LANDMARK_ID if unidentified (default) */
		int32_t id;
		/** The inverse of the observation covariance matrix (default:all zeros)
		 */
		mrpt::math::CMatrixDouble66 inf_matrix;

		/** Ctor with default values */
		TMeasurement();

		// Required because we contain Eigen
		// matrices
	};
	/** The list of observed features */
	std::deque<TMeasurement> sensedFeatures;

	/** The pose of the sensor on the robot/vehicle */
	mrpt::poses::CPose3D sensorPose;

	void getSensorPose(mrpt::poses::CPose3D& out_sensorPose)
		const override;	 // See base class docs.
	void setSensorPose(const mrpt::poses::CPose3D& newSensorPose)
		override;  // See base class docs.
	void getDescriptionAsText(
		std::ostream& o) const override;  // See base class docs

};	// End of class def.

}  // namespace mrpt::obs
