/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/obs/CObservationCANBusJ1939.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/hwdrivers/CGenericSensor.h>
#include <mrpt/comms/CSerialPort.h>

namespace mrpt::hwdrivers
{
/** This "software driver" implements the communication protocol for interfacing
 * a SICK LMS 2XX laser scanners through a standard RS232 serial port (or a
 * USB2SERIAL converter).
 *   The serial port is opened upon the first call to "doProcess" or
 * "initialize", so you must call "loadConfig" before
 *   this, or manually call "setSerialPort". Another alternative is to call the
 * base class method C2DRangeFinderAbstract::bindIO,
 *   but the "setSerialPort" interface is probably much simpler to use.
 *
 *   For an example of usage see the example in
 * "samples/SICK_laser_serial_test".
 *   See also the example configuration file for rawlog-grabber in
 * "share/mrpt/config_files/rawlog-grabber".
 *
 *  \code
 *  PARAMETERS IN THE ".INI"-LIKE CONFIGURATION STRINGS:
 * -------------------------------------------------------
 *   [supplied_section_name]
 *   COM_port_WIN = COM1   // Serial port to connect to
 *   COM_port_LIN = ttyS0
 *
 *   COM_baudRate = 38400 // Possible values: 9600 (default), 38400, 5000000
 *   mm_mode      = 1/0   // 1: millimeter mode, 0:centimeter mode (Default=0)
 *   FOV          = 180   // Field of view: 100 or 180 degrees (Default=180)
 *   resolution   =  50   // Scanning resolution, in units of 1/100 degree.
 * Valid values: 25,50,100 (Default=50)
 *
 *
 *   pose_x=0.21	// Laser range scaner 3D position in the robot (meters)
 *   pose_y=0
 *   pose_z=0.34
 *   pose_yaw=0	// Angles in degrees
 *   pose_pitch=0
 *   pose_roll=0
 *  \endcode
 *
 * \sa C2DRangeFinderAbstract
 * \ingroup mrpt_hwdrivers_grp
 */
class CCANBusReader : public mrpt::system::COutputLogger, public CGenericSensor
{
	DEFINE_GENERIC_SENSOR(CCANBusReader)

   private:
	/** Tries to open the com port and setup all the LMS protocol. Returns true
	 * if OK or already open. */
	bool tryToOpenComms(std::string* err_msg = nullptr);
	bool waitContinuousSampleFrame(
		uint8_t& out_prio, uint8_t& out_pdu_format, uint8_t& out_pdu_spec,
		uint8_t& out_src_address, uint8_t& out_data_length, uint16_t& out_pgn,
		std::vector<uint8_t>& out_data, std::vector<char>& out_raw_frame);

	/** Sends the specified speed to the CAN Converter. */
	bool sendCANBusReaderSpeed();
	/** Opens the CAN Channel */
	bool CANBusOpenChannel();
	/** Closes the CAN Channel */
	bool CANBusCloseChannel();
	bool CANBusAutoPoll();
	bool CANBusPoll();
	bool CANBusX1();
	bool setupSerialComms();
	bool queryVersion(bool printOutVersion = false);
	bool waitACK(uint16_t timeout_ms);
	bool waitForVersion(uint16_t timeout, bool printOutVersion = false);
	bool waitIncomingFrame(uint16_t timeout);

	bool sendCommandToCANReader(
		const uint8_t* cmd, const uint16_t cmd_len, bool wait = true);

	uint8_t m_received_frame_buffer[2000];

	/** If set to non-empty, the serial port will be attempted to be opened
	 * automatically when this class is first used to request data from the
	 * laser. */
	std::string m_com_port;
	/** Will be !=nullptr only if I created it, so I must destroy it at the end.
	 */
	mrpt::comms::CSerialPort* m_mySerialPort{nullptr};
	/** Baudrate: 9600, 38400, 500000 */
	int m_com_baudRate{57600};
	/** Default = 1 */
	unsigned int m_nTries_connect{1};
	unsigned int m_nTries_current{0};
	int m_canbus_speed{250000};
	bool m_canreader_timestamp{false};  // for future work
	bool m_CANBusChannel_isOpen{
		false};  // if the can bus channel is open or not

   protected:
	/** See the class documentation at the top for expected parameters */
	void loadConfig_sensorSpecific(
		const mrpt::config::CConfigFileBase& configSource,
		const std::string& iniSection) override;

   public:
	/** Constructor  */
	CCANBusReader();

	/** Destructor  */
	~CCANBusReader() override;

	/** Changes the serial port to connect to (call prior to 'doProcess'), for
	 * example "COM1" or "ttyS0".
	 *  This is not needed if the configuration is loaded with "loadConfig".
	 */
	void setSerialPort(const std::string& port) { m_com_port = port; }
	/** \sa setSerialPort */
	std::string getSerialPort() const { return m_com_port; }
	/** Changes the serial port baud rate (call prior to 'doProcess'); valid
	 * values are 9600,38400 and 500000.
	 *  This is not needed if the configuration is loaded with "loadConfig".
	 *  \sa getBaudRate */
	void setBaudRate(int baud) { m_com_baudRate = baud; }
	/** \sa setBaudRate */
	int getBaudRate() const { return m_com_baudRate; }
	/** Enables/Disables the addition of a timestamp according to the arrival
	 * time to the converter (default=false)
	 *  (call prior to 'doProcess') This is not needed if the configuration is
	 * loaded with "loadConfig".
	 */
	void setCANReaderTimeStamping(bool setTimestamp = false)
	{
		m_canreader_timestamp = setTimestamp;
	}
	bool getCANReaderTimeStamping() { return m_canreader_timestamp; }
	/** Sets the CAN reader speed when connecting to the CAN Bus
	 */
	void setCANReaderSpeed(const unsigned int speed) { m_canbus_speed = speed; }
	unsigned int getCANReaderSpeed() { return m_canbus_speed; }
	/** If performing several tries in ::initialize(), this is the current try
	 * loop number. */
	unsigned int getCurrentConnectTry() const { return m_nTries_current; }
	/** Specific laser scanner "software drivers" must process here new data
	 * from the I/O stream, and, if a whole scan has arrived, return it.
	 *  This method will be typically called in a different thread than other
	 * methods, and will be called in a timely fashion.
	 */
	void doProcessSimple(
		bool& outThereIsObservation,
		mrpt::obs::CObservationCANBusJ1939& outObservation,
		bool& hardwareError);

	/** Set-up communication with the laser.
	 *  Called automatically by rawlog-grabber.
	 *  If used manually, call after "loadConfig" and before "doProcess".
	 *
	 *  In this class this method does nothing, since the communications are
	 * setup at the first try from "doProcess" or "doProcessSimple".
	 */
	void initialize() override;

	void doProcess() override;

};  // End of class

}  // namespace mrpt::hwdrivers
