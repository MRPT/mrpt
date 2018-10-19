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
#include <mrpt/obs/CObservationIMU.h>

namespace mrpt::hwdrivers
{
enum GYRO_MODE
{
	RATE,
	INCREMENTAL_ANGLE,
	INTEGRATED_ANGLE
};
/** A class for interfacing KVH DSP 3000 gyroscope with an assynchronous serial
 *communication (product SN : 02-1222-01).
 *  It uses a serial port connection to the device. The class implements the
 *generic sensor class.
 *  See also the application "rawlog-grabber" for a ready-to-use application to
 *gather data from the scanner.
 *	 The generated observation is a CObservationIMU, but only the yaw angular
 *velocity and the absolute yaw position are
 *  are set in the vector CObservationIMU::rawMeasurements.
 *  The sensor process rate is imposed by hardware at 100Hz.
 *  For now, this sensor is only supported on posix system.
 *  \code
 *  PARAMETERS IN THE ".INI"-LIKE CONFIGURATION STRINGS:
 * -------------------------------------------------------
 *   [supplied_section_name]
 *    process_rate = 100				; MUST be 100 Hz.
 *    pose_x=0	    ; Sensor 3D position relative to the robot (meters)
 *    pose_y=0
 *    pose_z=0
 *    pose_yaw=0	; Angles in degrees
 *    pose_pitch=0
 *    pose_roll=0
 *    sensorLabel = <label> ; Label of the sensor
 *    COM_port_LIN	= /dev/ttyUSB0       ; COM PORT in LINUX
 *    operatingMode = <"rate"/"integrated", "incremental">  ; Default mode is
 *Rate.
 *  \endcode
 * In most of the communs applications, this class will be used as :
 * \code
 * CGyroKVHDSP3000 kvh;
 * /// ...
 * CConfigFile conf("conf.ini");
 * /// ...
 * kvh.loadConfig_sensorSpecific(conf, "KVH");
 * /// ...
 * while(1) {
 * 	kvh.doProcess();
 *		TListObservations rateObs;
 * 	kvh.getObservations(rateObs);
 *		// ....
 * \endcode
 * \ingroup mrpt_hwdrivers_grp
 */
class CGyroKVHDSP3000 : public hwdrivers::CGenericSensor
{
	DEFINE_GENERIC_SENSOR(CGyroKVHDSP3000)
   protected:
	/** This serial port will be attempted to be opened automatically when this
	 * class is first used to request data from the device.
	 * \sa comms::CSerialPort
	 */
	int m_COMbauds{38400};
	std::string m_com_port;

	mrpt::poses::CPose3D m_sensorPose;

	/** Search the port where the sensor is located and connect to it
	 */
	// bool searchPortAndConnect();

	/** The serial port connection */
	mrpt::comms::CSerialPort* m_serialPort;
	GYRO_MODE m_mode{RATE};
	bool m_firstInteration{true};

	mrpt::obs::CObservationIMU::Ptr m_observationGyro;

   public:
	/** Constructor
	 */
	CGyroKVHDSP3000();

	/** See the class documentation at the top for expected parameters */
	void loadConfig_sensorSpecific(
		const mrpt::config::CConfigFileBase& configSource,
		const std::string& iniSection) override;

	/** Destructor
	 */
	~CGyroKVHDSP3000() override;

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
	/** Send to the sensor the command 'Z' wich reset the integrated angle. (in
	 * both rate mode and incremental, this function has no effect) */
	void resetIncrementalAngle();
	void changeMode(GYRO_MODE _newMode);

};  // end of class

}  // namespace mrpt::hwdrivers
