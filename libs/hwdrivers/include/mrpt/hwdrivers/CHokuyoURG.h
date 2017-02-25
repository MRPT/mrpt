/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CHokuyoURG_H
#define CHokuyoURG_H

#include <mrpt/poses/CPose3D.h>
#include <mrpt/hwdrivers/C2DRangeFinderAbstract.h>
#include <mrpt/utils/circular_buffer.h>
#include <mrpt/gui/CDisplayWindow3D.h>

namespace mrpt
{
	namespace hwdrivers
	{
		/** This software driver implements the protocol SCIP-2.0 for interfacing HOKUYO URG, UTM and UXM laser scanners (USB or Ethernet)
		  *  Refer to the example code [HOKUYO_laser_test](http://www.mrpt.org/tutorials/mrpt-examples/example_hokuyo_urgutm_laser_scanner/) 
		  *  and to example rawlog-grabber [config files](https://github.com/MRPT/mrpt/tree/master/share/mrpt/config_files/rawlog-grabber)
		  *
		  *  See also the application "rawlog-grabber" for a ready-to-use application to gather data from the scanner.
		  *
		  *  \code
		  *  PARAMETERS IN THE ".INI"-LIKE CONFIGURATION STRINGS:
		  * -------------------------------------------------------
		  *   [supplied_section_name]
		  *    HOKUYO_motorSpeed_rpm=600
		  *    #HOKUYO_HS_mode   = false    // Optional (un-comment line if used): Set/unset the High-sensitivity mode (not on all models/firmwares!)
		  *
		  *    # Uncomment serial port or IP address, depending on the Hokuyo model (serial/USB vs. Ethernet):
		  *    COM_port_WIN = COM3       // Serial port name in Windows
		  *    COM_port_LIN = ttyS0      // Serial port name in GNU/Linux
		  *    #IP_DIR	=	192.168.0.10 // Uncommented this and "PORT_DIR" if the used HOKUYO is connected by Ethernet instead of USB
		  *    #PORT_DIR = 10940         // Default value: 10940
		  *
		  *    pose_x=0.21	// Laser range scaner 3D position in the robot (meters)
		  *    pose_y=0
		  *    pose_z=0.34
		  *    pose_yaw=0	// Angles in degrees
		  *    pose_pitch=0
		  *    pose_roll=0
		  *    
		  *    #disable_firmware_timestamp = true   // Uncomment to use PC time instead of laser time
		  *
		  *    # Optional: reduced FOV:
		  *    # reduced_fov  = 25 // Deg
		  *
		  *    #preview = true // Enable GUI visualization of captured data
		  *
		  *    # Optional: Exclusion zones to avoid the robot seeing itself:
		  *    #exclusionZone1_x = 0.20 0.30 0.30 0.20
		  *    #exclusionZone1_y = 0.20 0.30 0.30 0.20
		  *
		  *    # Optional: Exclusion zones to avoid the robot seeing itself:
		  *    #exclusionAngles1_ini = 20  // Deg
		  *    #exclusionAngles1_end = 25  // Deg
		  *
		  *  \endcode
		  * \ingroup mrpt_hwdrivers_grp
		  */
		class HWDRIVERS_IMPEXP CHokuyoURG : public C2DRangeFinderAbstract
		{
			DEFINE_GENERIC_SENSOR(CHokuyoURG)
		public:

			/** Used in CHokuyoURG::displayVersionInfo */
			struct TSensorInfo
			{
				std::string		model;			//!< The sensor model
				double			d_min,d_max;	//!< Min/Max ranges, in meters.
				int				scans_per_360deg;	//!< Number of measuremens per 360 degrees.
				int				scan_first,scan_last, scan_front;	//!< First, last, and front step of the scanner angular span.
				int				motor_speed_rpm;		//!< Standard motor speed, rpm.
			};

		private:
			int		m_firstRange,m_lastRange;   //!< The first and last ranges to consider from the scan.
			int		m_motorSpeed_rpm;           //!< The motor speed (default=600rpm)
			poses::CPose3D	m_sensorPose;       //!< The sensor 6D pose:
			mrpt::utils::circular_buffer<uint8_t> m_rx_buffer; //!< Auxiliary buffer for readings

			std::string     m_lastSentMeasCmd; //!< The last sent measurement command (MDXXX), including the last 0x0A.

			bool			m_highSensMode;  //!< High sensitivity [HS] mode (default: false)
			mrpt::gui::CDisplayWindow3DPtr m_win;

			/** Enables the SCIP2.0 protocol (this must be called at the very begining!).
			  * \return false on any error
			  */
			bool  enableSCIP20();

			/** Passes to 115200bps bitrate.
			  * \return false on any error
			  */
			bool  setHighBaudrate();

			/** Switchs the laser on.
			  * \return false on any error
			  */
			bool  switchLaserOn();

			/** Switchs the laser off
			  * \return false on any error
			  */
			bool  switchLaserOff();

			/** Changes the motor speed in rpm's (default 600rpm)
			  * \return false on any error
			  */
			bool  setMotorSpeed(int motoSpeed_rpm);

			/** Ask to the device, and print to the debug stream, details about the firmware version,serial number,...
			  * \return false on any error
			  */
			bool  displayVersionInfo(  );

			/** Ask to the device, and print to the debug stream, details about the sensor model.
			  *  It also optionally saves all the information in an user supplied data structure "out_data".
			  * \return false on any error
			  */
			bool  displaySensorInfo( CHokuyoURG::TSensorInfo * out_data = NULL );

			/** Start the scanning mode, using parameters stored in the object (loaded from the .ini file)
			  * After this command the device will start to send scans until "switchLaserOff" is called.
			  * \return false on any error
			  */
			bool  startScanningMode();

			/** Turns the laser on */
			void initialize();

			/** Waits for a response from the device.
			  * \return false on any error
			  */
			bool  receiveResponse(
					const char	*sentCmd_forEchoVerification,
					char	&rcv_status0,
					char	&rcv_status1,
					char	*rcv_data,
					int		&rcv_dataLength);


			/** Assures a minimum number of bytes in the input buffer, reading from the serial port only if required.
			  * \return false if the number of bytes are not available, even after trying to fetch more data from the serial port.
			  */
			bool assureBufferHasBytes(const size_t nDesiredBytes);

		public:
			/** Constructor
			  */
			CHokuyoURG();

			/** Destructor: turns the laser off */
			virtual ~CHokuyoURG();

			/** Specific laser scanner "software drivers" must process here new data from the I/O stream, and, if a whole scan has arrived, return it.
			  *  This method will be typically called in a different thread than other methods, and will be called in a timely fashion.
			  */
			void  doProcessSimple(
				bool							&outThereIsObservation,
				mrpt::obs::CObservation2DRangeScan	&outObservation,
				bool							&hardwareError );

			/** Enables the scanning mode (which may depend on the specific laser device); this must be called before asking for observations to assure that the protocol has been initializated.
			  * \return If everything works "true", or "false" if there is any error.
			  */
			bool  turnOn();

			/** Disables the scanning mode (this can be used to turn the device in low energy mode, if available)
			  * \return If everything works "true", or "false" if there is any error.
			  */
			bool  turnOff();

			/** Empties the RX buffers of the serial port */
			void purgeBuffers();

			/** If set to non-empty, the serial port will be attempted to be opened automatically when this class is first used to request data from the laser.  */
			void setSerialPort(const std::string &port_name) { m_com_port = port_name; }

			/** Set the ip direction and port to connect using Ethernet communication */
			void setIPandPort(const std::string &ip, const unsigned int &port) { m_ip_dir = ip; m_port_dir = port; }

			/** Returns the currently set serial port \sa setSerialPort */
			const std::string getSerialPort() { return m_com_port; }

			/** If called (before calling "turnOn"), the field of view of the laser is reduced to the given range (in radians), discarding the rest of measures.
			  *  Call with "0" to disable this reduction again (the default).
			  */
			void setReducedFOV(const double fov) { m_reduced_fov = fov; }

			/** Changes the high sensitivity mode (HS) (default: false)
			  * \return false on any error
			  */
			bool  setHighSensitivityMode(bool enabled);

			/** If true scans will capture intensity. (default: false)
			  * Should not be called while scanning.
			  * \return false on any error
			  */
			bool  setIntensityMode(bool enabled);

		protected:
			/** Returns true if there is a valid stream bound to the laser scanner, otherwise it first try to open the serial port "m_com_port"
			  */
			bool  checkCOMisOpen();

			double  		m_reduced_fov; //!< Used to reduce artificially the interval of scan ranges.

			std::string		m_com_port;		//!< If set to non-empty, the serial port will be attempted to be opened automatically when this class is first used to request data from the laser.

			std::string		m_ip_dir;	//!< If set to non-empty and m_port_dir too, the program will try to connect to a Hokuyo using Ethernet communication
			unsigned int	m_port_dir;	//!< If set to non-empty and m_ip_dir too, the program will try to connect to a Hokuyo using Ethernet communication

			/** The information gathered when the laser is first open */
			TSensorInfo		m_sensor_info;

			bool			m_I_am_owner_serial_port;

			uint32_t                 m_timeStartUI;	//!< Time of the first data packet, for synchronization purposes.
			int                      m_timeStartSynchDelay;  //!< Counter to discard to first few packets before setting the correspondence between device and computer timestamps.
			mrpt::system::TTimeStamp m_timeStartTT;
			bool                     m_disable_firmware_timestamp;
			bool                     m_intensity;  //!< Get intensity from lidar scan (default: false)

			/** See the class documentation at the top for expected parameters */
			void  loadConfig_sensorSpecific(
				const mrpt::utils::CConfigFileBase &configSource,
				const std::string	  &iniSection );

		};	// End of class

	} // End of namespace

} // End of namespace

#endif
