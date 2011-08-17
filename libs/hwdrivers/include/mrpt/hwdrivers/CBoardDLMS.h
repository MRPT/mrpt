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

#ifndef CBoardDLMS_H
#define CBoardDLMS_H

#include <mrpt/hwdrivers/CGenericSensor.h>
#include <mrpt/hwdrivers/CInterfaceFTDIMessages.h>
#include <mrpt/utils/CConfigFileBase.h>
#include <mrpt/poses/CPose3D.h>

namespace mrpt
{
	namespace hwdrivers
	{
		/** An interface to a custom board which interfaces two SICK laser scanners.
		  *  Implemented for the board v1.0 designed by 2008 @ ISA (University of Malaga).
		  *
		  *  \code
		  *  PARAMETERS IN THE ".INI"-LIKE CONFIGURATION STRINGS:
		  * -------------------------------------------------------
		  *   [DualLMS]
		  *	   driver			= CBoardDLMS
		  *	   process_rate		= 30		; Hz
		  *    USB_serialname	= DLMS-001
		  *
		  *    mPose_x			= 0	; Master laser range scaner 6D position on the robot (meters)
		  *	   mPose_y			= 0
          *    mPose_z			= 0
		  *    mPose_yaw		= 0
		  *    mPose_pitch		= 0
		  *    mPose_roll		= 0
		  *
		  *    sPose_x			= 0	; Slave laser range scaner 6D position on the robot (meters)
		  *	   sPose_y			= 0
          *    sPose_z			= 0
		  *    sPose_yaw		= 0
		  *    sPose_pitch		= 0
		  *    sPose_roll		= 0
		  *  \endcode
		  *
		  * \ingroup mrpt_hwdrivers_grp
 		  */
		class HWDRIVERS_IMPEXP CBoardDLMS : public hwdrivers::CInterfaceFTDIMessages, public CGenericSensor
		{
			DEFINE_GENERIC_SENSOR(CBoardDLMS)
		protected:
			/** A copy of the device serial number (to open the USB FTDI chip)
			  */
			std::string					m_usbSerialNumber;
			uint32_t					m_timeStartUI;
			mrpt::system::TTimeStamp	m_timeStartTT;

			mrpt::poses::CPose3D		m_mSensorPose, m_sSensorPose;

			/** Tries to connect to the USB device (if disconnected).
			  * \return True on connection OK, false on error.
			  */
			bool	checkConnectionAndConnect();

			bool	checkCRC( const std::vector<unsigned char> &frame );

			/** Loads specific configuration for the device from a given source of configuration parameters, for example, an ".ini" file, loading from the section "[iniSection]" (see utils::CConfigFileBase and derived classes)
			  *  See hwdrivers::CBoardDLMS for the possible parameters
			  */
			void  loadConfig_sensorSpecific(
				const mrpt::utils::CConfigFileBase &configSource,
				const std::string	  &iniSection );

		public:
			/** Constructor
			  * \param serialNumberUSBdevice The serial number (text) of the device to open.
			  *  The constructor will try to open the device. You can check if it failed calling "isOpen()".
			  */
			CBoardDLMS( );

			/** Destructor
			  */
			virtual ~CBoardDLMS();

			/** This method should be called periodically (at least at 1Hz to capture ALL the real-time data)
			*  It is thread safe, i.e. you can call this from one thread, then to other methods from other threads.
			*  This method processes data from the GPS and update the object state accordingly.
			*/
			void  doProcess();

			/** This method can or cannot be implemented in the derived class, depending on the need for it.
			  *  \exception This method must throw an exception with a descriptive message if some critical error is found.
			  */
			virtual void initialize();

			/** Query the firmware version on the device (can be used to test communications).
			  * \return true on success, false on communications errors or device not found.
			  */
			bool queryFirmwareVersion( std::string &out_firmwareVersion );

			/** Send a command to the DLMS Board
			  * \return true on success, false on communications errors or device not found.
			  */
			bool sendCommand( uint8_t command, std::vector<unsigned char> &response );

			bool queryTimeStamp( mrpt::system::TTimeStamp &tstamp );

		}; // end of class

	} // end of namespace
} // End of namespace

#endif
