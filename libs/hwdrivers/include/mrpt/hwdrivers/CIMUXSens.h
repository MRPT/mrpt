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

			/** Loads specific configuration for the device from a given source of configuration parameters, for example, an ".ini" file, loading from the section "[iniSection]" (see utils::CConfigFileBase and derived classes)
			  *  See hwdrivers::CIMUXSens for the possible parameters
			  */
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


