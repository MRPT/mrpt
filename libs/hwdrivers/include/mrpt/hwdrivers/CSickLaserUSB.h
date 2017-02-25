/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
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
		  *    //preview = true // Enable GUI visualization of captured data
		  *
		  *    // Optional: Exclusion zones to avoid the robot seeing itself:
		  *    //exclusionZone1_x = 0.20 0.30 0.30 0.20
		  *    //exclusionZone1_y = 0.20 0.30 0.30 0.20
		  *
		  *    // Optional: Exclusion zones to avoid the robot seeing itself:
		  *    //exclusionAngles1_ini = 20  // Deg
		  *    //exclusionAngles1_end = 25  // Deg
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
			/** See the class documentation at the top for expected parameters */
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
				mrpt::obs::CObservation2DRangeScan	&outObservation,
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
