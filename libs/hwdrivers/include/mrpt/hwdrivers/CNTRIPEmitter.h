/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CNTRIPEmitter_H
#define CNTRIPEmitter_H

#include <mrpt/hwdrivers/CNTRIPClient.h>
#include <mrpt/hwdrivers/CSerialPort.h>
#include <mrpt/hwdrivers/CGenericSensor.h>

namespace mrpt
{
	namespace hwdrivers
	{
		/** This "virtual driver" encapsulates a NTRIP client (see CNTRIPClient) but adds the functionality of dumping the received datastream to a given serial port.
		  *  Used within rawlog-grabber, along CGPSInterface, this class allows to build a powerful & simple RTK-capable GPS receiver system.
		  *
		  *  Therefore, this sensor will never "collect" any observation via the CGenericSensor interface.
		  *
		  *   See also the example configuration file for rawlog-grabber in "share/mrpt/config_files/rawlog-grabber".
		  *
		  *  \code
		  *  PARAMETERS IN THE ".INI"-LIKE CONFIGURATION STRINGS:
		  * -------------------------------------------------------
		  *   [supplied_section_name]
		  *   COM_port_WIN = COM1         // Serial port where the NTRIP stream will be dumped to.
		  *   COM_port_LIN = ttyUSB0
		  *   baudRate     = 38400   
		  *   #transmit_to_server = true   // (Default:true) Whether to send back to the TCP/IP caster all data received from the output serial port
		  *
		  *   server   = 143.123.9.129    // NTRIP caster IP
		  *   port     = 2101
		  *   mountpoint = MYPOINT23
		  *   //user = pepe            // User & password optional.
		  *   //password = loco
		  *
		  *  \endcode
		  *
		  *  The next picture summarizes existing MRPT classes related to GPS / GNSS devices (CGPSInterface, CNTRIPEmitter, CGPS_NTRIP):
		  *
		  *  <div align=center> <img src="mrpt_gps_classes_usage.png"> </div>
		  *
		  * \ingroup mrpt_hwdrivers_grp
		  * \sa CGPSInterface, CGPS_NTRIP, CNTRIPClient
		  */
		class HWDRIVERS_IMPEXP CNTRIPEmitter : public CGenericSensor
		{
			DEFINE_GENERIC_SENSOR(CNTRIPEmitter)

		private:
			CNTRIPClient::NTRIPArgs	m_ntrip_args;

			CNTRIPClient	m_client;  //!< The NTRIP comms object.
			CSerialPort		m_out_COM; //!< The output serial port.

			std::string     m_com_port;		//!< If set to non-empty, the serial port will be attempted to be opened automatically when this class is first used to request data from the laser.
			int             m_com_bauds;
			bool            m_transmit_to_server;

		protected:
			/** See the class documentation at the top for expected parameters */
			void  loadConfig_sensorSpecific(
				const mrpt::utils::CConfigFileBase &configSource,
				const std::string	  &iniSection );

		public:
			/** Constructor  */
			CNTRIPEmitter();

			/** Destructor  */
			virtual ~CNTRIPEmitter();

			/** Changes the serial port to connect to (call prior to 'doProcess'), for example "COM1" or "ttyS0".
			  *  This is not needed if the configuration is loaded with "loadConfig".
			  */
			void  setOutputSerialPort(const std::string &port) { m_com_port = port; }

			/** Set up the NTRIP communications, raising an exception on fatal errors.
			  *  Called automatically by rawlog-grabber.
			  *  If used manually, call after "loadConfig" and before "doProcess".
			  */
			void initialize();

			/** The main loop, which must be called in a timely fashion in order to process the incomming NTRIP data stream and dump it to the serial port.
			  *  This method is called automatically when used within rawlog-grabber.
			  */
			void doProcess(); 

			/** Exposes the NTRIP client object */
			CNTRIPClient & getNTRIPClient() { return m_client; }
			/** Exposes the NTRIP client object */
			const CNTRIPClient & getNTRIPClient() const { return m_client; }

		};	// End of class

	} // End of namespace
} // End of namespace

#endif
