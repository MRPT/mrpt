/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef CIMUXSens_MT4_H
#define CIMUXSens_MT4_H

#include <mrpt/hwdrivers/CGenericSensor.h>
#include <mrpt/poses/CPose3D.h>

namespace mrpt
{
	namespace hwdrivers
	{

		/** A class for interfacing XSens 4th generation Inertial Measuring Units (IMUs): MTi 10-series, MTi 100-series.
		  *  Usage considerations:
		  *    - In Windows, you only need to install XSens drivers.
		  *    - In Linux, this class requires the system libraries: libusb-1.0 & libudev (dev packages). Accessing USB devices may require
		  *      running the program as super user ("sudo"). To avoid that, Or, install <code> MRPT/scripts/52-xsens.rules </code> in <code>/etc/udev/rules.d/</code> to allow access to all users.
		  *
		  *  For the older 3rd generation devices, see the class mrpt::hwdrivers::CIMUXSens
		  *
		  *  \code
		  *  PARAMETERS IN THE ".INI"-LIKE CONFIGURATION STRINGS:
		  * -------------------------------------------------------
		  *   [supplied_section_name]
		  *    pose_x=0	    // Sensor 3D position relative to the robot (meters)
		  *    pose_y=0
		  *    pose_z=0
		  *    pose_yaw=0	// Angles in degrees
		  *    pose_pitch=0
		  *    pose_roll=0
		  *    sensorLabel = <label>   // Label of the sensor
		  *    ;sampleFreq  = 100  // The requested rate of sensor packets (default: 100Hz)
		  *    ; If a portname is not provided, the first found device will be opened:
		  *    ;portname_LIN	= USB002:005
		  *    ;portname_WIN	= \\?\usb#vid_2639&pid_0003#...
		  *    ;baudRate	    = 115200             ; Baudrate for communicating, only if the port is a COM port
		  *  \endcode
		  *
		  *  \note Set the environment variable "MRPT_HWDRIVERS_VERBOSE" to "1" to enable diagnostic information while using this class.
		  *
		  * \ingroup mrpt_hwdrivers_grp
 		  */
		class HWDRIVERS_IMPEXP CIMUXSens_MT4 : public hwdrivers::CGenericSensor
		{
			DEFINE_GENERIC_SENSOR(CIMUXSens_MT4)
		protected:
			int							m_port_bauds; //!< Baudrate, only for COM ports.
			std::string					m_portname;   //!< The USB or COM port name (if blank -> autodetect)
			int      m_sampleFreq;

			uint64_t					m_timeStartUI;
			mrpt::system::TTimeStamp	m_timeStartTT;

			mrpt::poses::CPose3D		m_sensorPose;

			void * /*DeviceClass */		m_dev_ptr;
			void * /*XsDeviceId */      m_devid_ptr;

			/** See the class documentation at the top for expected parameters */
			void  loadConfig_sensorSpecific(
				const mrpt::utils::CConfigFileBase &configSource,
				const std::string	  &iniSection );

		public:
			/** Constructor
			  */
			CIMUXSens_MT4( );

			/** Destructor
			  */
			virtual ~CIMUXSens_MT4();


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


