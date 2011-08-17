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
#ifndef CSickLaserUSB_H
#define CSickLaserUSB_H

#include <mrpt/hwdrivers/C2DRangeFinderAbstract.h>
#include <mrpt/hwdrivers/CInterfaceFTDI.h>

namespace mrpt
{
	namespace hwdrivers
	{
		/** This "software driver" implements the communication protocol for interfacing a SICK LMS2XX laser scanners through a custom USB RS-422 interface board.
		  *
		  *   NOTE that this class is for a custom hardware built at our lab (MAPIR, University of Malaga).
		  *   For a generic serial interface, see the class CSickLaserSerial.
		  *
		  *   This class does not need to be bind, i.e. you do not need to call C2DRangeFinderAbstract::bindIO. However, calling it will have not effect.
		  *   In this class the "bind" is ignored since it is designed for USB connections only, thus it internally generate the required object for simplicity of use.
		  *   The serial number of the USB device is used to open it on the first call to "doProcess", thus you must call "loadConfig" before this, or manually
		  *     call "setDeviceSerialNumber". The default serial number is "LASER001"
		  *
		  * Warning: Avoid defining an object of this class in a global scope if you want to catch all potential
		  *      exceptions during the constructors (like USB interface DLL not found, etc...)
		  *
		  *
		  *  \code
		  *  PARAMETERS IN THE ".INI"-LIKE CONFIGURATION STRINGS:
		  * -------------------------------------------------------
		  *   [supplied_section_name]
		  *   SICKUSB_serialNumber=LASER001
		  *   pose_x=0.21	; Laser range scaner 3D position in the robot (meters)
		  *   pose_y=0
		  *   pose_z=0.34
		  *   pose_yaw=0	; Angles in degrees
		  *   pose_pitch=0
		  *   pose_roll=0
		  *  \endcode
		  * \ingroup mrpt_hwdrivers_grp
		  *
		  */
		class HWDRIVERS_IMPEXP CSickLaserUSB : public C2DRangeFinderAbstract
		{
			DEFINE_GENERIC_SENSOR(CSickLaserUSB)

		private:
			CInterfaceFTDI		*m_usbConnection;
			std::string			m_serialNumber;

			uint32_t		m_timeStartUI;	//!< Time of the first data packet, for synchronization purposes.
			mrpt::system::TTimeStamp	m_timeStartTT;

			/** The sensor 6D pose:
			  */
			poses::CPose3D		m_sensorPose;

			bool 	checkControllerIsConnected();
			bool  	waitContinuousSampleFrame( std::vector<float> &ranges, unsigned char &LMS_status, uint32_t &out_board_timestamp, bool &is_mm_mode );

		protected:
			/** Loads specific configuration for the device from a given source of configuration parameters, for example, an ".ini" file, loading from the section "[iniSection]" (see utils::CConfigFileBase and derived classes)
			  *  See hwdrivers::CSickLaserUSB for the possible parameters
			  */
			void  loadConfig_sensorSpecific(
				const mrpt::utils::CConfigFileBase &configSource,
				const std::string	  &iniSection );

		public:
			/** Constructor
			  */
			CSickLaserUSB();

			/** Destructor
			  */
			virtual ~CSickLaserUSB();

			/** Changes the serial number of the device to open (call prior to 'doProcess')
			  */
			void  setDeviceSerialNumber(const std::string &deviceSerialNumber)
			{
				m_serialNumber = deviceSerialNumber;
			}

			/** Specific laser scanner "software drivers" must process here new data from the I/O stream, and, if a whole scan has arrived, return it.
			  *  This method will be typically called in a different thread than other methods, and will be called in a timely fashion.
			  */
			void  doProcessSimple(
				bool							&outThereIsObservation,
				mrpt::slam::CObservation2DRangeScan	&outObservation,
				bool							&hardwareError );


			/** Enables the scanning mode (in this class this has no effect).
			  * \return If everything works "true", or "false" if there is any error.
			  */
			bool  turnOn();

			/** Disables the scanning mode (in this class this has no effect).
			  * \return If everything works "true", or "false" if there is any error.
			  */
			bool  turnOff();

		};	// End of class

	} // End of namespace
} // End of namespace

#endif
