/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/hwdrivers/C2DRangeFinderAbstract.h>
#include <mrpt/comms/CClientTCPSocket.h>

namespace mrpt::hwdrivers
{
class CSICKTim561Eth : public C2DRangeFinderAbstract
{
	DEFINE_GENERIC_SENSOR(CSICKTim561Eth)
   public:
	/** Constructor.
	 * Note that there is default arguments, here you can customize IP Adress
	 * and TCP Port of your device
	 */
	CSICKTim561Eth(
		std::string _ip = std::string("192.168.0.1"),
		unsigned int _port = 2111);
	/** Deconstructor.
	 * Close communication with the device, and free memory.
	 */
	~CSICKTim561Eth() override;

	/** This function acquire a laser scan from the device. If an error occured,
	 * hardwareError will be set to true.
	 * The new laser scan will be stored in the outObservation argument.
	 *
	 * \exception This method throw exception if the frame received from the
	 * TIM 561 contain the following bad parameters:
	 * * Status is not OK
	 * * Data in the scan aren't DIST1(may be RSSIx or DIST2).
	 */
	void doProcessSimple(
		bool& outThereIsObservation,
		mrpt::obs::CObservation2DRangeScan& outObservation,
		bool& hardwareError) override;

	/**
	 * This method must be called before trying to get a laser scan.
	 */
	bool turnOn() override;

	/**
	 * This method could be called manually to stop communication with the
	 * device. Method is also called by destructor.
	 */
	bool turnOff() override;

	/**
	 * This method could be called manually to reboot the device.
	 */
	bool rebootDev();

	/**
	 * A method to set the sensor pose on the robot.
	 * Equivalent to setting the sensor pose via loading it from a config file.
	 */
	void setSensorPose(const mrpt::poses::CPose3D& _pose);

	/**
	 * This method must be called periodically. Period depend on the
	 * process_rate in the configuration file.
	 */
	void doProcess() override;

	/** Initialize the sensor according to the parameters previously read
	 * in the configuration file.
	 */
	void initialize() override;

   private:
	std::string m_ip;
	unsigned int m_port;
	mrpt::comms::CClientTCPSocket m_client;
	bool m_turnedOn{false};
	std::string m_cmd;
	bool m_connected{false};
	unsigned int m_scanFrequency;  // Scanning Hertz
	double m_angleResolution;  // Degrees [0.0001, 1]
	double m_startAngle;  // the minimum startAngle is -45 degree
	double m_stopAngle;  // the maximum stopAngle is 225 degree
	mrpt::poses::CPose3D m_sensorPose;
	double m_maxRange{10.0};  // 10 meter is the range official suggested
	double m_beamApperture;  // the range bewteen m_startAngle to m_stopAngle

	void generateCmd(const char* cmd);  // According to communication protocol
	bool checkIsConnected();  // Check the status about Lidar connection
	bool decodeLogIn(char* msg);
	bool decodeScanCfg(std::istringstream& stream);
	bool decodeScan(
		char* buf, mrpt::obs::CObservation2DRangeScan& outObservation);
	void sendCommand(const char* cmd);
	void roughPrint(char* msg);

   protected:
	/** Load sensor pose on the robot, or keep the default sensor pose.
	 */
	void loadConfig_sensorSpecific(
		const mrpt::config::CConfigFileBase& configSource,
		const std::string& iniSection) override;
};
}  // namespace mrpt::hwdrivers
