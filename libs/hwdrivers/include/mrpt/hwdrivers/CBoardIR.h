/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef CBoardIR_H
#define CBoardIR_H

#include <mrpt/obs/CObservationRange.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/hwdrivers/CSerialPort.h>
#include <mrpt/utils/CDebugOutputCapable.h>
#include <mrpt/hwdrivers/CGenericSensor.h>
#include <mrpt/obs/obs_frwds.h>

namespace mrpt
{
	namespace hwdrivers
	{
		/** A parser of NMEA commands, for connecting to a GPS by a serial port.
		  * This class also supports more advanced GPS equipped with RTK corrections. See the JAVAD/TopCon extra initialization parameters.
		  *
		  *  \code
		  *  PARAMETERS IN THE ".INI"-LIKE CONFIGURATION STRINGS:
		  * -------------------------------------------------------
		  *   [supplied_section_name]
		  *    COM_port_WIN = COM3
		  *    COM_port_LIN = ttyS0
		  *    baudRate     = 4800   // The baudrate of the communications (typ. 4800 bauds)
		  *    pose_x       = 0      // 3D position of the sensed point relative to the robot (meters)
		  *    pose_y       = 0
		  *    pose_z       = 0
		  *    customInit   =       // See below for possible values
		  *
		  *	   // The next parameters are optional and will be used only
		  *    // if customInit=="JAVAD" to enable/configure the usage of RTK corrections:
		  *    //JAVAD_rtk_src_port=/dev/ser/b
		  *    //JAVAD_rtk_src_baud=9600
		  *    //JAVAD_rtk_format=cmr
		  *
		  *  \endcode
		  *
		  * - customInit: Custom commands to send, depending on the sensor. Valid values are:
		  *		- "": Empty string
		  *		- "JAVAD": JAVAD or TopCon devices. Extra initialization commands will be sent.
		  *		- "TopCon": A synonymous with "JAVAD".
		  *
		  *  VERSIONS HISTORY:
		  *		-9/JUN/2006: First version (JLBC)
		  *		-4/JUN/2008: Added virtual methods for device-specific initialization commands.
		  *		-10/JUN/2008: Converted into CGenericSensor class (there are no inhirited classes anymore).
		  * \ingroup mrpt_hwdrivers_grp
		*/
		class HWDRIVERS_IMPEXP CBoardIR : public utils::CDebugOutputCapable, public CGenericSensor
		{
			DEFINE_GENERIC_SENSOR(CBoardIR)

		public:
			/** Constructor
			  * \param BUFFER_LENGTH The size of the communications buffer (default value should be fine always)
			  */
			CBoardIR( int BUFFER_LENGTH = 13 );

			/** Destructor
			  */
			virtual ~CBoardIR();

			/** This method tries to get a set of range measurements from the IR sensors.
			  * \param outObservation The output observation
			  * \param outThereIsObservation Will be true if an observation was sucessfully received.
			  * \param hardwareError Will be true if there's some important error, e.g. serial port can't be open.
			  */
			void  getObservation(
				bool							&outThereIsObservation,
				mrpt::obs::CObservationRange	&outObservation,
				bool							&hardwareError );

			// See docs in parent class
			void  doProcess();

			void  setSerialPortName(const std::string &COM_port);	//!< Set the serial port to use (COM1, ttyUSB0, etc).
			std::string getSerialPortName() const;					//!< Get the serial port to use (COM1, ttyUSB0, etc).

		protected:
			bool OnConnectionEstablished();

			CSerialPort		m_COM;

			poses::CPoint3D	m_sensorPose;

			std::string		m_customInit;

			/** The minimum range in meters (10cm).
			  */
			float			m_minRange;

			/** The maximum range in meters (80cm).
			  */
			float			m_maxRange;

			/** The poses of the IR: x[m] y[m] z[m] yaw[deg] pitch[deg] roll[deg]
			  *  Up to 6 devices, but you can put any number of devices (from 1 to 6).
			  */
			std::map<uint16_t,mrpt::math::TPose3D>	m_IRPoses;

			/** See the class documentation at the top for expected parameters */
			void  loadConfig_sensorSpecific(
				const mrpt::utils::CConfigFileBase &configSource,
				const std::string	  &iniSection );

		private:
			std::string 	m_COMname;
			int				m_COMbauds;

			/** Returns true if the COM port is already open, or try to open it in other case.
			  * \return true if everything goes OK, or false if there are problems opening the port.
			  */
			bool  tryToOpenTheCOM();

		}; // end class

	} // end namespace
} // end namespace

#endif
