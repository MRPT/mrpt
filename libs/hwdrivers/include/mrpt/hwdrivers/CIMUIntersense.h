/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/hwdrivers/CGenericSensor.h>
#include <mrpt/comms/CSerialPort.h>
#include <mrpt/poses/CPose3D.h>

namespace mrpt::hwdrivers
{
// clang-format off
/** A class for interfacing Intersense Inertial Measuring Units (IMUs).
  *  It connects to a InterSense inertiaCube 3 sensor and records inertial data.
  *  NOTE: This device provides:
  *		- Euler angles,
  *		- 2 angular velocties (body-frame and navigation-frame)
  *		- X,Y,Z velocity
  *		- 2 accelerations (body-frame and navigation-frame)
  *
  *		In order to record all this information within the 'rawMeasurements'
  *vector
  *in mrpt::obs::CObservationIMU, some of it had to be stored in positions which
  *weren't intended for the stored data (marked with *):
  *		- Euler angles --> rawMeasurements[IMU_YAW], rawMeasurements[IMU_PITCH],
  *rawMeasurements[IMU_ROLL]
  *		- Body-frame angular velocity --> rawMeasurements[IMU_YAW_VEL],
  *rawMeasurements[IMU_PITCH_VEL], rawMeasurements[IMU_ROLL_VEL]
  *		- * Nav-frame angular velocity --> rawMeasurements[IMU_MAG_X],
  *rawMeasurements[IMU_MAG_Y], rawMeasurements[IMU_MAG_Z]
  *		- XYZ velocity --> rawMeasurements[IMU_X_VEL],
  *rawMeasurements[IMU_Y_VEL],
  *rawMeasurements[IMU_Z_VEL]
  *		- Body-frame acceleration --> rawMeasurements[IMU_X_ACC],
  *rawMeasurements[IMU_Y_ACC], rawMeasurements[IMU_Z_ACC]
  *		- * Nav-frame acceleration --> rawMeasurements[IMU_X],
  *rawMeasurements[IMU_Y], rawMeasurements[IMU_Z]
  *		Be careful with this when using the grabbed mrpt::obs::CObservationIMU
  *data.
  *
  *  See also the application "rawlog-grabber" for a ready-to-use application to
  *gather data from this sensor.
  *
  *  \code
  *  PARAMETERS IN THE ".INI"-LIKE CONFIGURATION STRINGS:
  * -------------------------------------------------------
  *   [supplied_section_name]
  *    driver		= CIMUIntersense
  *	   sensorLabel	= <label>			; Label of the sensor
  *    pose_x		= 0					; [double] Sensor 3D position relative to the robot (meters)
  *    pose_y		= 0
  *    pose_z		= 0
  *    pose_yaw		= 0					; [double] Angles in degrees
  *    pose_pitch	= 0
  *    pose_roll	= 0
  *
  *	   sensitivity	= 10				; [int] Sensor sensitivity (see API documentation)
  *	   enhancement	= 2					; [int] Enhancement mode (see API documentation)
  *	   prediction	= 0					; [int] Prediction mode (see API documentation)
  *	   useBuffer	= 0					; [bool] {0,1} (unused by now) Whether or not use a buffer for storing old data (see API documentation)
  *
  * \endcode
  * \note Class introduced in MRPT 1.3.1
  * \ingroup mrpt_hwdrivers_grp
  */  // clang-format
class CIMUIntersense : public hwdrivers::CGenericSensor
{
	DEFINE_GENERIC_SENSOR(CIMUIntersense)
   protected:
	/** Opaque pointer to specifid iSense IMU structure */
	void* /* ISD_TRACKER_HANDLE* */ m_handles_ptr{nullptr};

	/** Timestamp management */
	uint32_t m_timeStartUI{0};
	mrpt::system::TTimeStamp m_timeStartTT;

	mrpt::poses::CPose3D m_sensorPose;
	int m_nSensors{0};

	/* Configurable parameters */
	uint32_t m_sensitivity{10};
	uint32_t m_enhancement{2};
	uint32_t m_prediction{0};
	bool m_useBuffer{false};

	/** Timeout counter (for internal use only) */
	unsigned int m_toutCounter{0};

	/** See the class documentation at the top for expected parameters */
	void loadConfig_sensorSpecific(
		const mrpt::config::CConfigFileBase& configSource,
		const std::string& iniSection) override;

   public:
	/** Constructor
	  */
	CIMUIntersense();

	/** Destructor
	  */
	~CIMUIntersense() override;

	/** This method will be invoked at a minimum rate of "process_rate" (Hz)
	  *  \exception This method must throw an exception with a descriptive
	 * message if some critical error is found.
	  */
	void doProcess() override;

	/** Turns on the iSense device and configure it for getting orientation data
	 */
	void initialize() override;

};  // end of class

}
