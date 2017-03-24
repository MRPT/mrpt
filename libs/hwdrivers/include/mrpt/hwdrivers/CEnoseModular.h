/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef CEnoseModular_H
#define CEnoseModular_H

#include <mrpt/hwdrivers/CInterfaceFTDI.h>
#include <mrpt/hwdrivers/CSerialPort.h>
#include <mrpt/hwdrivers/CGenericSensor.h>
#include <mrpt/obs/CObservationGasSensors.h>
#include <mrpt/utils/CConfigFileBase.h>


namespace mrpt
{
	namespace hwdrivers
	{
		/** A class for interfacing an e-NoseModular via a FTDI USB link.
		  *  Implemented for the Mdular board v1.0 designed by 2013 @ MAPIR (University of Malaga).
		  *
		  *  \code
		  *  PARAMETERS IN THE ".INI"-LIKE CONFIGURATION STRINGS:
		  * -------------------------------------------------------
		  *   [supplied_section_name]
		  *    USB_serialname=ENOSE001   // USB FTDI pipe: will open only if COM_port_* are not set or empty
		  *
		  *    COM_port_WIN = COM1       // Serial port to connect to.
		  *    COM_port_LIN = ttyS0
		  *
		  *    COM_baudRate = 115200
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
		  * \ingroup mrpt_hwdrivers_grp
 		  */
		class HWDRIVERS_IMPEXP CEnoseModular : public mrpt::hwdrivers::CGenericSensor
		{
			DEFINE_GENERIC_SENSOR(CEnoseModular)

		protected:
			/** A copy of the device serial number (to open the USB FTDI chip)
			  */
			std::string		m_usbSerialNumber;
			mrpt::system::TTimeStamp initial_timestamp;
			bool first_reading;

			std::string		m_COM_port;  //!< If not an empty string (default), will open that serial port, otherwise will try to open USB FTDI device "m_usbSerialNumber"
			unsigned int 	m_COM_baud;	 //!< Default=115200


			// Only one of these two streams will be !=NULL and open for each specific eNose board!
			/**  FTDI comms pipe (when not in serial port mode) */
			CInterfaceFTDI	*m_stream_FTDI;
			/**  Serial port comms */
			CSerialPort		*m_stream_SERIAL;

			/** The 3D pose of the master + N slave eNoses on the robot (meters & radians) */
			std::vector<float>	enose_poses_x,enose_poses_y,enose_poses_z,enose_poses_yaw,enose_poses_pitch,enose_poses_roll;

			/** Tries to connect to the USB device (if disconnected).
			  * \return NULL on error, otherwise a stream to be used for comms.
			  */
			mrpt::utils::CStream*	checkConnectionAndConnect();

			/** See the class documentation at the top for expected parameters */
			void  loadConfig_sensorSpecific(
				const mrpt::utils::CConfigFileBase &configSource,
				const std::string			&section );

			/** Purge the Serial/FTDI buffer */
			void purgeBuffers();

		public:
			/** Constructor
			  * \param serialNumberUSBdevice The serial number (text) of the device to open.
			  *  The constructor will try to open the device. You can check if it failed calling "isOpen()".
			  */
			CEnoseModular( );

			/** Destructor
			  */
			virtual ~CEnoseModular();

			/** Request the master eNose the latest readings from all the eNoses.
			  *  The output observation contains a valid timestamp and 3D positions if "loadConfig" has been called previously.
			  * \return true if OK, false if there were any error.
			  */
			bool getObservation( mrpt::obs::CObservationGasSensors &outObservation );


			// See docs in parent class
			void  doProcess();		



			/** If not an empty string, will open that serial port, otherwise will try to open USB FTDI device "m_usbSerialNumber"
			  *  The default is an empty string. Example strings: "COM1", "ttyUSB0", ...
			  */
			inline void setSerialPort(const std::string &port) { m_COM_port = port; }
			inline std::string getSerialPort() const { return m_COM_port; }

			/** Set the serial port baud rate (default: 115200) */
			inline void setSerialPortBaud(unsigned int baud) { m_COM_baud=baud; }
			inline unsigned int getSerialPortBaud() const { return m_COM_baud; }


		}; // end of class
	} // end of namespace
} // end of namespace


#endif
