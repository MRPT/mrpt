/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/hwdrivers/CNTRIPClient.h>
#include <mrpt/comms/CSerialPort.h>
#include <mrpt/hwdrivers/CGenericSensor.h>
#include <mrpt/system/CTicTac.h>
#include <fstream>

namespace mrpt::hwdrivers
{
/** This "virtual driver" encapsulates a NTRIP client (see CNTRIPClient) but
 * adds the functionality of dumping the received datastream to a given serial
 * port.
 *  Used within rawlog-grabber, along CGPSInterface, this class allows to build
 * a powerful & simple RTK-capable GPS receiver system.
 *
 *  Therefore, this sensor will never "collect" any observation via the
 * CGenericSensor interface.
 *
 *   See also the example configuration file for rawlog-grabber in
 * "share/mrpt/config_files/rawlog-grabber".
 *
 *  \code
 *  PARAMETERS IN THE ".INI"-LIKE CONFIGURATION STRINGS:
 * -------------------------------------------------------
 *   [supplied_section_name]
 *   COM_port_WIN = COM1         // Serial port where the NTRIP stream will be
 * dumped to.
 *   COM_port_LIN = ttyUSB0
 *   baudRate     = 38400
 *   #transmit_to_server = true   // (Default:true) Whether to send back to the
 * TCP/IP caster all data received from the output serial port
 *   #raw_output_file_prefix = raw_ntrip_data    // If provided, save raw data
 * from the NTRIP to a file, useful for post-processing. In this case, not
 * having a serial port configured (commented out) is a valid configuration.
 *
 *   server   = 143.123.9.129    // NTRIP caster IP
 *   port     = 2101
 *   mountpoint = MYPOINT23
 *   //user = pepe            // User & password optional.
 *   //password = loco
 *
 *  \endcode
 *
 *  The next picture summarizes existing MRPT classes related to GPS / GNSS
 * devices (CGPSInterface, CNTRIPEmitter, CGPS_NTRIP):
 *
 *  <div align=center> <img src="mrpt_gps_classes_usage.png"> </div>
 *
 * \ingroup mrpt_hwdrivers_grp
 * \sa CGPSInterface, CGPS_NTRIP, CNTRIPClient
 */
class CNTRIPEmitter : public CGenericSensor
{
	DEFINE_GENERIC_SENSOR(CNTRIPEmitter)

   private:
	CNTRIPClient::NTRIPArgs m_ntrip_args;

	/** The NTRIP comms object. */
	CNTRIPClient m_client;
	/** The output serial port. */
	mrpt::comms::CSerialPort m_out_COM;

	/** If set to non-empty, the serial port will be attempted to be opened
	 * automatically when this class is first used to request data from the
	 * laser. */
	std::string m_com_port;
	int m_com_bauds{38400};
	bool m_transmit_to_server{true};
	std::string m_raw_output_file_prefix;
	std::ofstream m_raw_output_file_stream;
	mrpt::system::CTicTac m_rate_timer;
	size_t m_rate_count{0};

   protected:
	/** See the class documentation at the top for expected parameters */
	void loadConfig_sensorSpecific(
		const mrpt::config::CConfigFileBase& configSource,
		const std::string& iniSection) override;

   public:
	/** Constructor  */
	CNTRIPEmitter();

	/** Destructor  */
	~CNTRIPEmitter() override;

	/** Changes the serial port to connect to (call prior to 'doProcess'), for
	 * example "COM1" or "ttyS0".
	 *  This is not needed if the configuration is loaded with "loadConfig".
	 */
	void setOutputSerialPort(const std::string& port) { m_com_port = port; }
	std::string getOutputSerialPort() const { return m_com_port; }
	void setRawOutputFilePrefix(const std::string& outfile)
	{
		m_raw_output_file_prefix = outfile;
	}
	std::string getRawOutputFilePrefix() const
	{
		return m_raw_output_file_prefix;
	}

	/** Set up the NTRIP communications, raising an exception on fatal errors.
	 *  Called automatically by rawlog-grabber.
	 *  If used manually, call after "loadConfig" and before "doProcess".
	 */
	void initialize() override;

	/** The main loop, which must be called in a timely fashion in order to
	 * process the incomming NTRIP data stream and dump it to the serial port.
	 *  This method is called automatically when used within rawlog-grabber.
	 */
	void doProcess() override;

	/** Exposes the NTRIP client object */
	CNTRIPClient& getNTRIPClient() { return m_client; }
	/** Exposes the NTRIP client object */
	const CNTRIPClient& getNTRIPClient() const { return m_client; }
};  // End of class

}  // namespace mrpt::hwdrivers
