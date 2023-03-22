/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/comms/CSerialPort.h>
#include <mrpt/hwdrivers/CGenericSensor.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/poses/CPose3D.h>

namespace mrpt::hwdrivers
{
/** A driver for Taobotics IMU.
 *
 * Supported models: HFI-A9, HFI-B9, HFI-B6.
 *
 *  \code
 *  PARAMETERS IN THE ".INI"-LIKE CONFIGURATION STRINGS:
 * -------------------------------------------------------
 *   [supplied_section_name]
 *    process_rate = 500   # (Hz) must be larger than the sensor rate
 *    pose_x=0	    # Sensor 3D position relative to the robot (meters)
 *    pose_y=0
 *    pose_z=0
 *    pose_yaw=0	# Angles in degrees
 *    pose_pitch=0
 *    pose_roll=0
 *    sensorLabel = name   # Label of the sensor
 *    serialPort  = /dev/ttyUSB0
 *  \endcode
 *
 * If used programmatically, this class will be used as:
 *
 * \code
 * CTaoboticsIMU imu;
 * /// ...
 * CConfigFile conf("conf.ini");
 * /// ...
 * imu.loadConfig_sensorSpecific(conf, "IMU");
 * /// ...
 * while(1) {
 * 	imu.doProcess();
 *	TListObservations rateObs;
 * 	imu.getObservations(rateObs);
 *	// ....
 * \endcode
 * \ingroup mrpt_hwdrivers_grp
 */
class CTaoboticsIMU : public hwdrivers::CGenericSensor
{
	DEFINE_GENERIC_SENSOR(CTaoboticsIMU)

   public:
	CTaoboticsIMU();
	~CTaoboticsIMU() override;

	/** See the class documentation at the top for expected parameters */
	void loadConfig_sensorSpecific(
		const mrpt::config::CConfigFileBase& configSource,
		const std::string& iniSection) override;

	/** This method will be invoked at a minimum rate of "process_rate" (Hz)
	 *  \exception This method must throw an exception with a descriptive
	 * message if some critical error is found.
	 */
	void doProcess() override;

	/** Turns on the KVH DSP 3000 device and configure it for getting
	 * orientation data. you must have called loadConfig_sensorSpecific before
	 * calling this function.
	 */
	void initialize() override;

   protected:
	/** This serial port will be attempted to be opened automatically when this
	 * class is first used to request data from the device.
	 * \sa comms::CSerialPort
	 */
	int m_baudRate = 38400;
	std::string m_com_port;
	mrpt::poses::CPose3D m_sensorPose;

	/** The serial port connection */
	std::unique_ptr<mrpt::comms::CSerialPort> m_serialPort;

	mrpt::obs::CObservationIMU::Ptr m_observationGyro;

};	// end of class

}  // namespace mrpt::hwdrivers
