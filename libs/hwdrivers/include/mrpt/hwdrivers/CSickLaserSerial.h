/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CSickLaserSerial_H
#define CSickLaserSerial_H

#include <mrpt/hwdrivers/C2DRangeFinderAbstract.h>
#include <mrpt/hwdrivers/CSerialPort.h>
#include <mrpt/math/lightweight_geom_data.h>

namespace mrpt
{
	namespace hwdrivers
	{
		/** This "software driver" implements the communication protocol for interfacing a SICK LMS 2XX laser scanners through a standard RS232 serial port (or a USB2SERIAL converter).
		  *   The serial port is opened upon the first call to "doProcess" or "initialize", so you must call "loadConfig" before
		  *   this, or manually call "setSerialPort". Another alternative is to call the base class method C2DRangeFinderAbstract::bindIO,
		  *   but the "setSerialPort" interface is probably much simpler to use.
		  *
		  *   For an example of usage see the example in "samples/SICK_laser_serial_test".
		  *   See also the example configuration file for rawlog-grabber in "share/mrpt/config_files/rawlog-grabber".
		  *
		  *  \code
		  *  PARAMETERS IN THE ".INI"-LIKE CONFIGURATION STRINGS:
		  * -------------------------------------------------------
		  *   [supplied_section_name]
		  *   COM_port_WIN = COM1   // Serial port to connect to
		  *   COM_port_LIN = ttyS0
		  *
		  *   COM_baudRate = 38400 // Possible values: 9600 (default), 38400, 5000000
		  *   mm_mode      = 1/0   // 1: millimeter mode, 0:centimeter mode (Default=0)
		  *   FOV          = 180   // Field of view: 100 or 180 degrees (Default=180)
		  *   resolution   =  50   // Scanning resolution, in units of 1/100 degree. Valid values: 25,50,100 (Default=50)
		  *   //skip_laser_config  = true // (Default:false) If true, doesn't send the initialization commands to the laser and go straight to capturing
		  *
		  *   pose_x=0.21	// Laser range scaner 3D position in the robot (meters)
		  *   pose_y=0
		  *   pose_z=0.34
		  *   pose_yaw=0	// Angles in degrees
		  *   pose_pitch=0
		  *   pose_roll=0
		  *
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
		  *
		  * \sa C2DRangeFinderAbstract
		  * \ingroup mrpt_hwdrivers_grp
		  */
		class HWDRIVERS_IMPEXP CSickLaserSerial : public C2DRangeFinderAbstract
		{
			DEFINE_GENERIC_SENSOR(CSickLaserSerial)

		private:
			bool	m_mm_mode;
			int		m_scans_FOV; //!< 100 or 180 deg
			int		m_scans_res; //!< 1/100th of deg: 100, 50 or 25

			/** The sensor 6D pose: */
			mrpt::math::TPose3D		m_sensorPose;

			static int			CRC16_GEN_POL;


			bool    tryToOpenComms(std::string *err_msg=NULL);	//!< Tries to open the com port and setup all the LMS protocol. Returns true if OK or already open.
			bool  	waitContinuousSampleFrame( std::vector<float> &ranges, unsigned char &LMS_status, bool &is_mm_mode );


			bool LMS_setupSerialComms();	//!< Assures laser is connected and operating at 38400, in its case returns true.
			bool LMS_setupBaudrate(int baud);	//!< Send a command to change the LMS comms baudrate, return true if ACK is OK. baud can be: 9600, 19200, 38400, 500000
			bool LMS_statusQuery();	//!< Send a status query and wait for the answer. Return true on OK.
			bool LMS_waitACK(uint16_t timeout_ms); //!< Returns false if timeout
			bool LMS_waitIncomingFrame(uint16_t timeout); //!< Returns false if timeout
			bool LMS_sendMeasuringMode_cm_mm();  //!< Returns false on error
			bool LMS_startContinuousMode();
			bool LMS_endContinuousMode();

            bool SendCommandToSICK(const uint8_t *cmd,const uint16_t cmd_len); //!< Send header+command-data+crc and waits for ACK. Return false on error.

			uint8_t			m_received_frame_buffer[2000];

			std::string		m_com_port;		//!< If set to non-empty, the serial port will be attempted to be opened automatically when this class is first used to request data from the laser.
			CSerialPort		*m_mySerialPort; //!< Will be !=NULL only if I created it, so I must destroy it at the end.
			int             m_com_baudRate; //!< Baudrate: 9600, 38400, 500000
			unsigned int    m_nTries_connect; //!< Default = 1
			unsigned int    m_nTries_current;
            bool            m_skip_laser_config; //!< If true, doesn't send the initialization commands to the laser and go straight to capturing

		protected:
			/** See the class documentation at the top for expected parameters */
			void  loadConfig_sensorSpecific(
				const mrpt::utils::CConfigFileBase &configSource,
				const std::string	  &iniSection );

		public:
			/** Constructor  */
			CSickLaserSerial();

			/** Destructor  */
			virtual ~CSickLaserSerial();

			/** Changes the serial port to connect to (call prior to 'doProcess'), for example "COM1" or "ttyS0".
			  *  This is not needed if the configuration is loaded with "loadConfig".
			  */
			void  setSerialPort(const std::string &port) { m_com_port = port; }

			/** \sa setSerialPort */
			std::string getSerialPort() const { return m_com_port; }

			/** Changes the serial port baud rate (call prior to 'doProcess'); valid values are 9600,38400 and 500000.
			  *  This is not needed if the configuration is loaded with "loadConfig".
			  *  \sa getBaudRate */
			void setBaudRate(int baud) {
					m_com_baudRate = baud;
			}
			/** \sa setBaudRate */
			int getBaudRate() const { return m_com_baudRate; }


			/** Enables/Disables the millimeter mode, with a greater accuracy but a shorter range (default=false)
			  *  (call prior to 'doProcess') This is not needed if the configuration is loaded with "loadConfig".
			  */
			void setMillimeterMode(bool mm_mode=true) { m_mm_mode = mm_mode; }

			/** Set the scanning field of view - possible values are 100 or 180 (default)
			  *  (call prior to 'doProcess') This is not needed if the configuration is loaded with "loadConfig".
			  */
			void setScanFOV(int fov_degrees) { m_scans_FOV = fov_degrees; }
			int getScanFOV() const { return m_scans_FOV; }

			/** Set the scanning resolution, in units of 1/100 degree - Possible values are 25, 50 and 100, for 0.25, 0.5 (default) and 1 deg.
			  *  (call prior to 'doProcess') This is not needed if the configuration is loaded with "loadConfig".
			  */
			void setScanResolution(int res_1_100th_degree) { m_scans_res=res_1_100th_degree; }
			int getScanResolution() const { return m_scans_res; }

            /** If performing several tries in ::initialize(), this is the current try loop number. */
			unsigned int getCurrentConnectTry() const { return m_nTries_current; }


			/** Specific laser scanner "software drivers" must process here new data from the I/O stream, and, if a whole scan has arrived, return it.
			  *  This method will be typically called in a different thread than other methods, and will be called in a timely fashion.
			  */
			void  doProcessSimple(
				bool							&outThereIsObservation,
				mrpt::obs::CObservation2DRangeScan	&outObservation,
				bool							&hardwareError );


			/** Set-up communication with the laser.
			  *  Called automatically by rawlog-grabber.
			  *  If used manually, call after "loadConfig" and before "doProcess".
			  *
			  *  In this class this method does nothing, since the communications are setup at the first try from "doProcess" or "doProcessSimple".
			  */
			void initialize();

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
