/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
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

		/** A class for interfacing XSens 3rd generation Inertial Measuring Units (IMUs), the "XSens MTi" model.
		  *  It uses a serial port or USB-to-serial adapter to communicate to the device, so no special drivers are needed.
		  *  For the more recent 4th generation devices, see the class mrpt::hwdrivers::CIMUXSens_MT4
		  *
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


