/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/obs/CObservation.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/serialization/CSerializable.h>

namespace mrpt::obs
{
/** This observation represents a number of range-bearing value pairs, each one
 * for a detected landmark, which optionally can have identification IDs.
 *  This class can manage sensors that detect landmarks in a 2D plane (e.g. a
 * laser scanner) or in the 3D space (e.g. a camera). There are
 *  two direction angles: yaw (azimuth) and pitch (negative elevation). For 2D
 * sensors, the pitch must be always set to 0.
 * See CObservationBearingRange::validCovariances for the instructions to fill
 * the uncertainty covariances.
 * \sa CObservation
 * \ingroup mrpt_obs_grp
 */
class CObservationBearingRange : public CObservation
{
	DEFINE_SERIALIZABLE(CObservationBearingRange, mrpt::obs)

   public:
	CObservationBearingRange() = default;

	float minSensorDistance{0}, maxSensorDistance{0};  //! Information about the
	//! sensor: Ranges, in meters
	//! (0: there is no limits)
	/** Information about the sensor: The "field-of-view" of the sensor, in
	 * radians (for yaw ). */
	float fieldOfView_yaw = mrpt::d2f(180.0_deg);
	/** Information about the sensor: The "field-of-view" of the sensor, in
	 * radians (for pitch ). */
	float fieldOfView_pitch = mrpt::d2f(90.0_deg);

	/** The position of the sensor on the robot.
	 */
	mrpt::poses::CPose3D sensorLocationOnRobot;

	/** Each one of the measurements:
	 */
	struct TMeasurement
	{
		/** The sensed landmark distance, in meters. */
		float range;

		/** The sensed landmark direction, in radians, measured as the yaw
		 * (azimuth) and pitch (negative elevation).
		 *  Set pitch to zero for 2D sensors.
		 * See mrpt::poses::CPose3D for a definition of the 3D angles.
		 */
		float yaw, pitch;

		/** The ID of the sensed beacon, or INVALID_LANDMARK_ID (-1) if the
		 * sensor does not identify the landmark. */
		int32_t landmarkID;

		/** The covariance matrix of the landmark, with variable indices [0,1,2]
		 * being [range,yaw,pitch]. */
		mrpt::math::CMatrixDouble33 covariance;
	};

	using TMeasurementList = std::vector<TMeasurement>;

	/** The list of observed ranges:  */
	TMeasurementList sensedData;

	/** True: The individual 3x3 covariance matrices must be taken into account,
	 * false (default): All the measurements have identical, diagonal 3x3
	 * covariance matrices given by the values
	 * sensor_std_range,sensor_std_yaw,sensor_std_pitch.
	 */
	bool validCovariances{false};

	/** Taken into account only if validCovariances=false: the standard
	 * deviation of the sensor noise model for range,yaw and pitch (in meters
	 * and radians).
	 *  If validCovariances=true, these 3 values are ignored and the individual
	 * 3x3 covariance matrices contain the actual uncertainties for each of the
	 * detected landmarks.
	 */
	float sensor_std_range{0}, sensor_std_yaw{0}, sensor_std_pitch{0};

	/** Prints out the contents of the object.
	 */
	void debugPrintOut();

	// See base class docs
	void getSensorPose(mrpt::poses::CPose3D& out_sensorPose) const override
	{
		out_sensorPose = sensorLocationOnRobot;
	}
	void setSensorPose(const mrpt::poses::CPose3D& newSensorPose) override
	{
		sensorLocationOnRobot = newSensorPose;
	}
	void getDescriptionAsText(std::ostream& o) const override;

};  // End of class def.

}  // namespace mrpt::obs
