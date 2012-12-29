/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                                 |
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
