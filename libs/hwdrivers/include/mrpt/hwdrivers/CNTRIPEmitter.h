/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
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
