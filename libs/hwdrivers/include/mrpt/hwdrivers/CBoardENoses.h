/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2010  University of Malaga                           |
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

#ifndef CBoardENoses_H
#define CBoardENoses_H

#include <mrpt/hwdrivers/CInterfaceFTDIMessages.h>
#include <mrpt/hwdrivers/CGenericSensor.h>

#include <mrpt/slam/CObservationGasSensors.h>
#include <mrpt/utils/CConfigFileBase.h>


namespace mrpt
{
	namespace hwdrivers
	{
		/** A class for interfacing an e-Noses via a FTDI USB link.
		  *  Implemented for the board v1.0 designed by 2007 @ ISA (University of Malaga).
		  *
		  *  \code
		  *  PARAMETERS IN THE ".INI"-LIKE CONFIGURATION STRINGS:
		  * -------------------------------------------------------
		  *   [supplied_section_name]
		  *    USB_serialname=ENOSE001
		  *
		  *    ; 3D position (in meters) of the master +slave eNoses
		  *    enose_poses_x=<MASTER X> <SLAVE#1 X> <SLAVE#2 X> <SLAVE#3 X>...
		  *    enose_poses_y=<MASTER Y> <SLAVE#1 Y> <SLAVE#2 Y> <SLAVE#3 Y>...
		  *    enose_poses_z=<MASTER Z> <SLAVE#1 Z> <SLAVE#2 Z> <SLAVE#3 Z>...
		  *
		  *    ; 3D pose angles (in degrees) of the master +slave eNoses
		  *    enose_poses_yaw=<MASTER YAW> <SLAVE#1 YAW> <SLAVE#2 YAW> <SLAVE#3 YAW>...
		  *    enose_poses_pitch=<MASTER PITCH> <SLAVE#1 PITCH> <SLAVE#2 PITCH> <SLAVE#3 PITCH>...
		  *    enose_poses_roll=<MASTER ROLL> <SLAVE#1 ROLL> <SLAVE#2 ROLL> <SLAVE#3 ROLL>...
		  *
		  *  \endcode
		  *
 		  */
		class HWDRIVERS_IMPEXP CBoardENoses : public mrpt::hwdrivers::CGenericSensor, public mrpt::hwdrivers::CInterfaceFTDIMessages
		{
			DEFINE_GENERIC_SENSOR(CBoardENoses)

		protected:
			/** A copy of the device serial number (to open the USB FTDI chip)
			  */
			std::string		m_usbSerialNumber;
			std::string		m_sensorLabel;
			mrpt::system::TTimeStamp initial_timestamp;
			bool first_reading;

			/** The 3D pose of the master + N slave eNoses on the robot (meters & radians)
			  */
			vector_float	enose_poses_x,enose_poses_y,enose_poses_z,enose_poses_yaw,enose_poses_pitch,enose_poses_roll;

			/** Tries to connect to the USB device (if disconnected).
			  * \return True on connection OK, false on error.
			  */
			bool	checkConnectionAndConnect();

			/** Loads specific configuration for the device from a given source of configuration parameters, for example, an ".ini" file, loading from the section "[iniSection]" (see utils::CConfigFileBase and derived classes)
			  *  See hwdrivers::CBoardENoses for the possible parameters
			  */
			void  loadConfig_sensorSpecific(
				const mrpt::utils::CConfigFileBase &configSource,
				const std::string			&section );

		public:
			/** Constructor
			  * \param serialNumberUSBdevice The serial number (text) of the device to open.
			  *  The constructor will try to open the device. You can check if it failed calling "isOpen()".
			  */
			CBoardENoses( );

			/** Destructor
			  */
			virtual ~CBoardENoses();

			/** Query the firmware version on the device (can be used to test communications).
			  * \return true on success, false on communications errors or device not found.
			  */
			bool	queryFirmwareVersion( std::string &out_firmwareVersion );

			/** Request the master eNose the latest readings from all the eNoses.
			  *  The output observation contains a valid timestamp and 3D positions if "loadConfig" has been called previously.
			  * \return true if OK, false if there were any error.
			  */
			bool getObservation( mrpt::slam::CObservationGasSensors &outObservation );


			/** This method should be called periodically (at least at 1Hz to capture ALL the real-time data)
			*  It is thread safe, i.e. you can call this from one thread, then to other methods from other threads.
			*/
			void  doProcess();

			/** Tries to open the camera, after setting all the parameters with a call to loadConfig.
			  *  \exception This method must throw an exception with a descriptive message if some critical error is found.
			  */
			virtual void initialize();



		}; // end of class
	} // end of namespace
} // end of namespace


#endif


