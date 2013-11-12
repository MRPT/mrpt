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
		  *    - In Linux, this class only requires MRPT to be built with libusb-1.0 support. Accessing USB devices may require
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
		  *	   sensorLabel = <label>   // Label of the sensor
		  *    # If a portname is not provided, the first found device will be opened:
		  *	   #portname_LIN	= USB002:005
		  *	   #portname_WIN	= \\?\usb#vid_2639&pid_0003#...
		  *	   #baudRate	    = 115200             ; Baudrate for communicating, only if the port is a COM port
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


