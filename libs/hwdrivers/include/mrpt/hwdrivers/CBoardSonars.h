/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CBoardSonars_H
#define CBoardSonars_H

#include <mrpt/hwdrivers/CInterfaceFTDIMessages.h>
#include <mrpt/hwdrivers/CGenericSensor.h>
#include <mrpt/utils/CConfigFileBase.h>
#include <mrpt/obs/CObservationRange.h>

namespace mrpt
{
	namespace hwdrivers
	{
		/**   This "software driver" implements the communication protocol for interfacing a Ultrasonic range finder SRF10 through a custom USB board.
		  *
		  *   In this class the "bind" is ignored since it is designed for USB connections only, thus it internally generate the required object for simplicity of use.
		  *   The serial number of the USB device is used to open it on the first call to "doProcess", thus you must call "loadConfig" before this, or manually
		  *     call "setDeviceSerialNumber". The default serial number is "SONAR001"
		  *
		  * Warning: Avoid defining an object of this class in a global scope if you want to catch all potential
		  *      exceptions during the constructors (like USB interface DLL not found, etc...)
		  *
		  *  \code
		  *  PARAMETERS IN THE ".INI"-LIKE CONFIGURATION STRINGS:
		  * -------------------------------------------------------
		  *   [supplied_section_name]
		  *   USB_serialNumber=SONAR001
		  *   gain=6			; Value between 0 and 16, for analog gains between 40 and 700.
		  *   maxRange=4.0		; In meters, used for device internal timer.
		  *   minTimeBetweenPings=0.3  ; In seconds
		  *
		  *   ; The order in which sonars will be fired, indexed by their I2C addresses [0,15]
		  *   ;  Up to 16 devices, but you can put any number of devices (from 1 to 16).
		  *   firingOrder=0 1 2 3
		  *
		  *
		  *  \endcode
		  *
		  * \ingroup mrpt_hwdrivers_grp
		  */
		class HWDRIVERS_IMPEXP CBoardSonars : public hwdrivers::CInterfaceFTDIMessages, public CGenericSensor
		{
			DEFINE_GENERIC_SENSOR(CBoardSonars)

			public:
				/** Constructor
				  */
				CBoardSonars();

				/** Destructor
				  */
				virtual ~CBoardSonars(){}

				/** Query the firmware version on the device (can be used to test communications).
				  * \return true on success, false on communications errors or device not found.
				  */
				bool queryFirmwareVersion( std::string &out_firmwareVersion );

				/** Request the latest range measurements.
				  * \return true on success, false on communications errors or device not found.
				  */
				bool getObservation( mrpt::obs::CObservationRange &obs );

				/** Requests a command of "change address" for a given SRF10 device.
				  *  currentAddress and newAddress are the I2C addresses in the range 0 to 15 (mapped to 0xE0 to 0xFE internally).
				  * \return true on success, false on communications errors or device not found.
				  */
				bool programI2CAddress( uint8_t currentAddress, uint8_t newAddress );

				// See docs in parent class
				void  doProcess();

		protected:
				/** A copy of the device serial number (to open the USB FTDI chip)
				  */
				std::string		m_usbSerialNumber;

				/** A value between 0 and 16, for gains between 40 and 700 (not linear).
				  */
				uint8_t			m_gain;

				/** The maximum range in meters, used for the internal device timer (value between 4cm and 11m).
				  */
				float			m_maxRange;

				/** The order in which sonars will be fired, indexed by their I2C addresses [0,15].
				  *  Up to 16 devices, but you can put any number of devices (from 1 to 16).
				  */
				std::vector<int32_t>		m_firingOrder;

				/** The individual gains of the sonars, indexed by their I2C addresses [0,15].
				  *  Up to 16 devices, but you can put any number of devices (from 1 to 16).
				  */
				std::map<uint16_t,int32_t>	m_sonarGains;

				/** The poses of the sonars: x[m] y[m] z[m] yaw[deg] pitch[deg] roll[deg]
				  *  Up to 16 devices, but you can put any number of devices (from 1 to 16).
				  */
				std::map<uint16_t,mrpt::math::TPose3D>	m_sonarPoses;

				/** The minimum time between sonar pings (in seconds).
				  */
				float			m_minTimeBetweenPings;

				/** Tries to connect to the USB device (if disconnected).
				  * \return True on connection OK, false on error.
				  */
				bool	checkConnectionAndConnect();

				/** Sends the configuration (max range, gain,...) to the USB board. Used internally after a successfull connection.
				  * \return true on success, false on communications errors or device not found.
				  */
				bool	sendConfigCommands();

				/** Loads specific configuration for the device from a given source of configuration parameters, for example, an ".ini" file,
				  *  loading from the section "[iniSection]" (see utils::CConfigFileBase and derived classes)
				  *  See hwdrivers::CBoardSonars for the possible parameters
				  */
				void  loadConfig_sensorSpecific(	const mrpt::utils::CConfigFileBase &configSource,
									const std::string	  &iniSection );



		};	// End of class
	} // End of namespace
} // End of namespace


#endif
