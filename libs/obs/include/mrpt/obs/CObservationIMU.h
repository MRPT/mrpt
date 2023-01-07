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

#include <array>

namespace mrpt::obs
{
/** Symbolic names for the indices of IMU data (refer to
 * mrpt::obs::CObservationIMU)
 * \ingroup mrpt_obs_grp
 */
enum TIMUDataIndex
{
	/** x-axis acceleration (local/vehicle frame) (m/sec<sup>2</sup>) */
	IMU_X_ACC = 0,
	/** y-axis acceleration (local/vehicle frame) (m/sec<sup>2</sup>) */
	IMU_Y_ACC,
	/** z-axis acceleration (local/vehicle frame) (m/sec<sup>2</sup>) */
	IMU_Z_ACC,
	/** yaw angular velocity (local/vehicle frame) (rad/sec) */
	IMU_YAW_VEL,
	/** angular velocity - z (local/vehicle frame) (rad/sec) */
	IMU_WZ = IMU_YAW_VEL,
	/** pitch angular velocity (local/vehicle frame) (rad/sec) */
	IMU_PITCH_VEL,
	/** angular velocity - y (local/vehicle frame) (rad/sec) */
	IMU_WY = IMU_PITCH_VEL,
	/** roll angular velocity (local/vehicle frame) (rad/sec) */
	IMU_ROLL_VEL,
	/** angular velocity - x (local/vehicle frame) (rad/sec) */
	IMU_WX = IMU_ROLL_VEL,
	/** x-axis velocity (global/navigation frame) (m/sec) */
	IMU_X_VEL,
	/** y-axis velocity (global/navigation  frame) (m/sec) */
	IMU_Y_VEL,
	/** z-axis velocity (global/navigation  frame) (m/sec) */
	IMU_Z_VEL,
	/** orientation yaw absolute value (global/navigation frame) (rad) */
	IMU_YAW,
	/** orientation pitch absolute value (global/navigation frame) (rad) */
	IMU_PITCH,
	/** orientation roll absolute value (global/navigation frame) (rad) */
	IMU_ROLL,
	/** x absolute value (global/navigation frame) (meters) */
	IMU_X,
	/** y absolute value (global/navigation frame) (meters) */
	IMU_Y,
	/** z absolute value (global/navigation frame) (meters) */
	IMU_Z,
	/** x magnetic field value (local/vehicle frame) (gauss) */
	IMU_MAG_X,
	/** y magnetic field value (local/vehicle frame) (gauss) */
	IMU_MAG_Y,
	/** z magnetic field value (local/vehicle frame) (gauss) */
	IMU_MAG_Z,
	/** air pressure (Pascals) */
	IMU_PRESSURE,
	/** altitude from an altimeter (meters) */
	IMU_ALTITUDE,
	/** temperature (degrees Celsius) */
	IMU_TEMPERATURE,
	/** Orientation Quaternion X (global/navigation frame) */
	IMU_ORI_QUAT_X,
	/** Orientation Quaternion Y (global/navigation frame) */
	IMU_ORI_QUAT_Y,
	/** Orientation Quaternion Z (global/navigation frame) */
	IMU_ORI_QUAT_Z,
	/** Orientation Quaternion W (global/navigation frame) */
	IMU_ORI_QUAT_W,
	/** yaw angular velocity (global/navigation frame) (rad/sec) */
	IMU_YAW_VEL_GLOBAL,
	/** pitch angular velocity (global/navigation frame) (rad/sec) */
	IMU_PITCH_VEL_GLOBAL,
	/** roll angular velocity (global/navigation frame) (rad/sec) */
	IMU_ROLL_VEL_GLOBAL,
	/** x-axis acceleration (global/navigation frame) (m/sec<sup>2</sup>) */
	IMU_X_ACC_GLOBAL,
	/** y-axis acceleration (global/navigation frame) (m/sec<sup>2</sup>) */
	IMU_Y_ACC_GLOBAL,
	/** z-axis acceleration (global/navigation frame) (m/sec<sup>2</sup>) */
	IMU_Z_ACC_GLOBAL,

	// Always leave this last value to reflect the number of enum values
	COUNT_IMU_DATA_FIELDS
};

/** This class stores measurements from an Inertial Measurement Unit (IMU)
 * (attitude estimation, raw gyroscope and accelerometer values), altimeters or
 * magnetometers.
 *
 *  The order of the values in each entry of
 * mrpt::obs::CObservationIMU::rawMeasurements is defined as symbolic names in
 * the enum mrpt::obs::TIMUDataIndex.
 *  Check it out also for reference on the unit and the coordinate frame used
 * for each value.
 *
 * \sa CObservation
 * \ingroup mrpt_obs_grp
 */
class CObservationIMU : public CObservation
{
	DEFINE_SERIALIZABLE(CObservationIMU, mrpt::obs)

   public:
	CObservationIMU();

	~CObservationIMU() override = default;

	/** The pose of the sensor on the robot. */
	mrpt::poses::CPose3D sensorPose;

	/** Each entry in this vector is true if the corresponding data index
	 * contains valid data (the IMU unit supplies that kind of data).
	 *  See the top of this page for the meaning of the indices.
	 * Initial value: all set to false.
	 */
	std::array<bool, mrpt::obs::COUNT_IMU_DATA_FIELDS> dataIsPresent;

	/** The accelerometer and/or gyroscope measurements taken by the IMU at the
	 * given timestamp.
	 * \sa dataIsPresent, CObservation::timestamp
	 * Initial value: all zeros.
	 */
	std::array<double, mrpt::obs::COUNT_IMU_DATA_FIELDS> rawMeasurements;

	/** Sets a given data type, and mark it as present. \sa has(), set() */
	void set(TIMUDataIndex idx, double value)
	{
		rawMeasurements.at(idx) = value;
		dataIsPresent.at(idx) = true;
	}

	/** Gets a given data type, throws if not set. \sa has(), get() */
	double get(TIMUDataIndex idx) const
	{
		ASSERTMSG_(dataIsPresent.at(idx), "Trying to access non-set value");
		return rawMeasurements.at(idx);
	}
	/** Returns true if the given data type is set. \sa set(), get() */
	bool has(TIMUDataIndex idx) const { return dataIsPresent.at(idx); }

	// See base class docs
	void getSensorPose(mrpt::poses::CPose3D& out_sensorPose) const override
	{
		out_sensorPose = sensorPose;
	}
	void setSensorPose(const mrpt::poses::CPose3D& newSensorPose) override
	{
		sensorPose = newSensorPose;
	}
	void getDescriptionAsText(std::ostream& o) const override;

	// See base class docs:
	bool exportTxtSupported() const override { return true; }
	std::string exportTxtHeader() const override;
	std::string exportTxtDataRow() const override;

};	// End of class def.

}  // namespace mrpt::obs
