/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
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

#ifndef CBoardIR_H
#define CBoardIR_H

#include <mrpt/slam/CObservationRange.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/hwdrivers/CSerialPort.h>
#include <mrpt/utils/CDebugOutputCapable.h>
#include <mrpt/hwdrivers/CGenericSensor.h>

namespace mrpt
{
	namespace slam { class CObservationGPS; }

	namespace hwdrivers
	{
		/** A parser of NMEA commands, for connecting to a GPS by a serial port.
		  * This class also supports more advanced GPS equipped with RTK corrections. See the JAVAD/TopCon extra initialization parameters.
		  *
		  *  \code
		  *  PARAMETERS IN THE ".INI"-LIKE CONFIGURATION STRINGS:
		  * -------------------------------------------------------
		  *   [supplied_section_name]
		  *    COM_port_WIN = COM3
		  *    COM_port_LIN = ttyS0
		  *    baudRate     = 4800   // The baudrate of the communications (typ. 4800 bauds)
		  *    pose_x       = 0      // 3D position of the sensed point relative to the robot (meters)
		  *    pose_y       = 0
		  *    pose_z       = 0
		  *    customInit   =       // See below for possible values
		  *
		  *	   // The next parameters are optional and will be used only
		  *    // if customInit=="JAVAD" to enable/configure the usage of RTK corrections:
		  *    //JAVAD_rtk_src_port=/dev/ser/b
		  *    //JAVAD_rtk_src_baud=9600
		  *    //JAVAD_rtk_format=cmr
		  *
		  *  \endcode
		  *
		  * - customInit: Custom commands to send, depending on the sensor. Valid values are:
		  *		- "": Empty string
		  *		- "JAVAD": JAVAD or TopCon devices. Extra initialization commands will be sent.
		  *		- "TopCon": A synonymous with "JAVAD".
		  *
		  *  VERSIONS HISTORY:
		  *		-9/JUN/2006: First version (JLBC)
		  *		-4/JUN/2008: Added virtual methods for device-specific initialization commands.
		  *		-10/JUN/2008: Converted into CGenericSensor class (there are no inhirited classes anymore).
		  * \ingroup mrpt_hwdrivers_grp
		*/
		class HWDRIVERS_IMPEXP CBoardIR : public utils::CDebugOutputCapable, public CGenericSensor
		{
			DEFINE_GENERIC_SENSOR(CBoardIR)

		public:
			/** Constructor
			  * \param BUFFER_LENGTH The size of the communications buffer (default value should be fine always)
			  */
			CBoardIR( int BUFFER_LENGTH = 13 );

			/** Destructor
			  */
			virtual ~CBoardIR();

			/** This method tries to get a set of range measurements from the IR sensors.
			  * \param outObservation The output observation
			  * \param outThereIsObservation Will be true if an observation was sucessfully received.
			  * \param hardwareError Will be true if there's some important error, e.g. serial port can't be open.
			  */
			void  getObservation(
				bool							&outThereIsObservation,
				mrpt::slam::CObservationRange	&outObservation,
				bool							&hardwareError );

			/** This method should be called periodically (at least at 1Hz to capture ALL the real-time data)
			*  It is thread safe, i.e. you can call this from one thread, then to other methods from other threads.
			*  This method processes data from the GPS and update the object state accordingly.
			*/
			void  doProcess();

			void  setSerialPortName(const std::string &COM_port);	//!< Set the serial port to use (COM1, ttyUSB0, etc).
			std::string getSerialPortName() const;					//!< Get the serial port to use (COM1, ttyUSB0, etc).

		protected:
			bool OnConnectionEstablished();

			CSerialPort		m_COM;

			poses::CPoint3D	m_sensorPose;

			std::string		m_customInit;

			/** The minimum range in meters (10cm).
			  */
			float			m_minRange;

			/** The maximum range in meters (80cm).
			  */
			float			m_maxRange;

			/** The poses of the IR: x[m] y[m] z[m] yaw[deg] pitch[deg] roll[deg]
			  *  Up to 6 devices, but you can put any number of devices (from 1 to 6).
			  */
			std::map<uint16_t,mrpt::math::TPose3D>	m_IRPoses;

			/** Loads specific configuration for the device from a given source of configuration parameters, for example, an ".ini" file, loading from the section "[iniSection]" (see utils::CConfigFileBase and derived classes)
			  *  See hwdrivers::CGPSInterface for the possible parameters
			  */
			void  loadConfig_sensorSpecific(
				const mrpt::utils::CConfigFileBase &configSource,
				const std::string	  &iniSection );

		private:
			std::string 	m_COMname;
			int				m_COMbauds;

			/** Returns true if the COM port is already open, or try to open it in other case.
			  * \return true if everything goes OK, or false if there are problems opening the port.
			  */
			bool  tryToOpenTheCOM();

		}; // end class

	} // end namespace
} // end namespace

#endif
