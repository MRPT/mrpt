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

#ifndef CIMUXSens_H
#define CIMUXSens_H


#include <mrpt/hwdrivers/CGenericSensor.h>
#include <mrpt/hwdrivers/CSerialPort.h>

#include <mrpt/poses/CPose3D.h>

namespace mrpt
{
	namespace hwdrivers
	{

		/** A class for interfacing Inertial Measuring Units (IMUs) of the type "XSens MTi"
		  *  It uses a serial port connection to the device. The class implements the generic
		  *  See also the application "rawlog-grabber" for a ready-to-use application to gather data from the scanner.
		  *
		  *  \code
		  *  PARAMETERS IN THE ".INI"-LIKE CONFIGURATION STRINGS:
		  * -------------------------------------------------------
		  *   [supplied_section_name]
		  *    pose_x=0	    ; Sensor 3D position relative to the robot (meters)
		  *    pose_y=0
		  *    pose_z=0
		  *    pose_yaw=0	; Angles in degrees
		  *    pose_pitch=0
		  *    pose_roll=0
		  *	   sensorLabel = <label> ; Label of the sensor
		  *	   COM_port_LIN	= /dev/ttyUSB0       ; COM PORT in LINUX (optional. If not provided, the system will search the connected port)
		  *	   COM_port_WIN	= COM1               ; COM PORT in Windows (optional. If not provided, the system will search the connected port)
		  *	   baudRate	                         ; Baudrate for communicating with the COM port (mandatory for Linux)
		  *						                    (for Windows, if COM_port_WIN is not provided, this value is ignored)
		  *  \endcode
		  * \ingroup mrpt_hwdrivers_grp
 		  */
		class HWDRIVERS_IMPEXP CIMUXSens : public hwdrivers::CGenericSensor
		{
			DEFINE_GENERIC_SENSOR(CIMUXSens)
		protected:

			/** This serial port will be attempted to be opened automatically when this class is first used to request data from the device.
			  * \sa hwdrivers::CSerialPort
			  */
			int							m_COMbauds;
			std::string					m_com_port;
			uint64_t					m_timeStartUI;
			mrpt::system::TTimeStamp	m_timeStartTT;

			mrpt::poses::CPose3D		m_sensorPose;

			/** Search the port where the sensor is located and connect to it
			  */
			bool 			searchPortAndConnect();

			//CSerialPort		m_serial_port;			//!< The serial port connection

			void * /*xsens::Cmt3 */		m_cmt3_ptr;
			void * /*CmtDeviceId */		m_deviceId_ptr;
			unsigned int				m_toutCounter;				//!< Timeout counter (for internal use only)

			/** See the class documentation at the top for expected parameters */
			void  loadConfig_sensorSpecific(
				const mrpt::utils::CConfigFileBase &configSource,
				const std::string	  &iniSection );

		public:
			/** Constructor
			  */
			CIMUXSens( );

			/** Destructor
			  */
			virtual ~CIMUXSens();


			/** This method will be invoked at a minimum rate of "process_rate" (Hz)
			  *  \exception This method must throw an exception with a descriptive message if some critical error is found.
			  */
			void doProcess();

			/** Turns on the xSens device and configure it for getting orientation data */
			void initialize();

		}; // end of class

	} // end of namespace
} // end of namespace

#endif


