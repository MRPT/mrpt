/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
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
		  *
		  *   server   = 143.123.9.129    // NTRIP caster IP
		  *   port     = 2101
		  *   mountpoint = MYPOINT23
		  *   //user = pepe            // User & password optional.
		  *   //password = loco
		  *
		  *  \endcode
		  *
		  * \ingroup mrpt_hwdrivers_grp
		  * \sa CGPSInterface, CNTRIPClient
		  */
		class HWDRIVERS_IMPEXP CNTRIPEmitter : public CGenericSensor
		{
			DEFINE_GENERIC_SENSOR(CNTRIPEmitter)

		private:
			CNTRIPClient::NTRIPArgs	m_ntrip_args;

			CNTRIPClient	m_client;  //!< The NTRIP comms object.
			CSerialPort		m_out_COM; //!< The output serial port.

			std::string		m_com_port;		//!< If set to non-empty, the serial port will be attempted to be opened automatically when this class is first used to request data from the laser.
			int				m_com_bauds;

		protected:
			/** Loads specific configuration for the device from a given source of configuration parameters, for example, an ".ini" file, loading from the section "[iniSection]" (see utils::CConfigFileBase and derived classes)
			  *  See hwdrivers::CNTRIPEmitter for the possible parameters
			  */
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

		};	// End of class

	} // End of namespace
} // End of namespace

#endif
