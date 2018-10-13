/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/poses/CPose3D.h>
#include <mrpt/hwdrivers/C2DRangeFinderAbstract.h>
#include <mrpt/containers/circular_buffer.h>

namespace mrpt::hwdrivers
{
/** Interfaces a Robo Peak LIDAR laser scanner.
 *
 *  See the example "samples/RoboPeakLidar_laser_test" and the application
 * "rawlog-grabber" for a ready-to-use application to gather data from the
 * scanner.
 *
 *  \code
 *  PARAMETERS IN THE ".INI"-LIKE CONFIGURATION STRINGS:
 * -------------------------------------------------------
 *   [supplied_section_name]
 *    COM_port_WIN = COM3
 *    COM_port_LIN = ttyS0
 *    pose_x=0	// Laser range scaner 3D position in the robot (meters)
 *    pose_y=0
 *    pose_z=0
 *    pose_yaw=0	// Angles in degrees
 *    pose_pitch=0
 *    pose_roll=0
 *
 *    //preview = true // Enable GUI visualization of captured data
 *
 *    // Optional: Exclusion zones to avoid the robot seeing itself:
 *    //exclusionZone1_x = 0.20 0.30 0.30 0.20
 *    //exclusionZone1_y = 0.20 0.30 0.30 0.20
 *
 *    // Optional: Exclusion zones to avoid the robot seeing itself:
 *    //exclusionAngles1_ini = 20  // Deg
 *    //exclusionAngles1_end = 25  // Deg
 *
 *  \endcode
 * \note Class introduced in MRPT 1.2.2
 * \ingroup mrpt_hwdrivers_grp
 */
class CRoboPeakLidar : public C2DRangeFinderAbstract
{
	DEFINE_GENERIC_SENSOR(CRoboPeakLidar)
   public:
	/** Constructor */
	CRoboPeakLidar();
	/** Destructor: turns the laser off. */
	~CRoboPeakLidar() override;

	/** Attempts to connect and turns the laser on. Raises an exception on
	 * error. */
	void initialize() override;

	// See base class docs
	void doProcessSimple(
		bool& outThereIsObservation,
		mrpt::obs::CObservation2DRangeScan& outObservation,
		bool& hardwareError) override;

	/** If set to non-empty, the serial port will be attempted to be opened
	 * automatically when this class is first used to request data from the
	 * laser.  */
	void setSerialPort(const std::string& port_name);
	/** Returns the currently set serial port \sa setSerialPort */
	const std::string getSerialPort() { return m_com_port; }
	/** See base class docs */
	bool turnOn() override;
	/** See base class docs */
	bool turnOff() override;

	/** Returns true if the device is connected & operative */
	bool getDeviceHealth() const;

	/** Closes the comms with the laser. Shouldn't have to be directly needed by
	 * the user */
	void disconnect();

   protected:
	/** The sensor 6D pose: */
	poses::CPose3D m_sensorPose;
	std::string m_com_port;
	int m_com_port_baudrate{115200};
	void* m_rplidar_drv{nullptr};  // Opaque "RPlidarDriver*"

	/** Returns true if communication has been established with the device. If
	 * it's not,
	 *  try to create a comms channel.
	 * \return false on error.
	 */
	bool checkCOMMs();

	/** See the class documentation at the top for expected parameters */
	void loadConfig_sensorSpecific(
		const mrpt::config::CConfigFileBase& configSource,
		const std::string& iniSection) override;

};  // End of class

}  // namespace mrpt::hwdrivers
