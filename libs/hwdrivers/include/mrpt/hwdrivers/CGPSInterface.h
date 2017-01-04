/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef CGPSInterface_H
#define CGPSInterface_H

#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/hwdrivers/CSerialPort.h>
#include <mrpt/utils/COutputLogger.h>
#include <mrpt/utils/CFileOutputStream.h>
#include <mrpt/utils/TEnumType.h>
#include <mrpt/hwdrivers/CGenericSensor.h>
#include <mrpt/utils/circular_buffer.h>
#include <mrpt/obs/obs_frwds.h>

namespace mrpt
{
	namespace hwdrivers
	{
		/** A class capable of reading GPS/GNSS/GNSS+IMU receiver data, from a serial port or from any input stream, 
		  *  and \b parsing the ASCII/binary stream into indivual messages \b stored in mrpt::obs::CObservationGPS objects.
		  *
		  * Typical input streams are serial ports or raw GPS log files. By default, the serial port selected by CGPSInterface::setSerialPortName()
		  * or as set in the configuration file will be open upon call to CGenericSensor::initialize(). 
		  * Alternatively, an external stream can be bound with CGPSInterface::bindStream() before calling CGenericSensor::initialize(). 
		  * This feature can be used to parse commands from a file, a TCP/IP stream, a memory block, etc.
		  *
		  * The parsers in the enum type CGPSInterface::PARSERS are supported as parameter `parser` in the 
		  * configuration file below or in method CGPSInterface::setParser():
		  *  - `NONE`: Do not try to parse the messages into CObservation's. Only useful if combined with `raw_dump_file_prefix`
		  *  - `AUTO`: Try to automatically identify the format of incomming data.
		  *  - `NMEA` (NMEA 0183, ASCII messages): Default parser. Supported frames: GGA, RMC,... See full list of messages in children of mrpt::obs::gnss::gnss_message
		  *  - `NOVATEL_OEM6` (Novatel OEM6, binary frames): Supported frames: BESTPOS,... Note that receiving a correct IONUTC msg is required for a correct timestamping of subsequent frames. See full list of messages in children of mrpt::obs::gnss::gnss_message
		  *
		  * See available parameters below, and an example config file for rawlog-grabber [here](https://github.com/MRPT/mrpt/blob/master/share/mrpt/config_files/rawlog-grabber/gps.ini)
		  *
		  *  \code
		  *  PARAMETERS IN THE ".INI"-LIKE CONFIGURATION STRINGS:
		  * -------------------------------------------------------
		  * [supplied_section_name]
		  *
		  *  # Serial port configuration:
		  *  COM_port_WIN = COM3
		  *  COM_port_LIN = ttyUSB0
		  *  baudRate     = 4800   // The baudrate of the communications (typ. 4800 or 9600 bauds)
		  *
		  *  # (Default:true) Whether to append the GNNS message type to CObservation `sensorLabel` field
		  *  sensor_label_append_msg_type = true
		  *
		  *  # Select a parser for GNSS data:
		  *  # Up-to-date list of supported parsers available in http://reference.mrpt.org/devel/classmrpt_1_1hwdrivers_1_1_c_g_p_s_interface.html
		  *  parser =  AUTO
		  *
		  *  # If uncommented and non-empty, raw binary/ascii data received from the serial port will be also dumped 
		  *  # into a file named after this prefix, plus date/time and extension `.gps`.
		  *  #raw_dump_file_prefix = RAWGPS
		  *
		  *  # 3D position (and orientation, for GNSS+IMUs) of the sensed point (antenna phase center) relative to the vehicle/robot frame:
		  *  pose_x       = 0      // (meters)
		  *  pose_y       = 0
		  *  pose_z       = 0
		  *  pose_yaw     = 0      // (deg)
		  *  pose_pitch   = 0
		  *  pose_roll    = 0
		  *
		  *  # Optional: list of custom commands to be sent to the GNSS receiver to set it up.
		  *  # An arbitrary number of commands can be defined, but their names must be "setup_cmd%d" starting at "1". 
		  *  # Commands will be sent by index order. Binary commands instead of ASCII strings can be set programatically, not from a config file.
		  *  # custom_cmds_delay   = 0.1   // (Default=0.1) Delay in seconds between consecutive set-up commands
		  *  # custom_cmds_append_CRLF = true    // (Default:true) Append "\r\n" to each command
		  *  # setup_cmd1 = XXXXX
		  *  # setup_cmd2 = XXXXX
		  *  # setup_cmd3 = XXXXX
		  *
		  *  # Optional: list of commands to be sent upon disconnection (e.g. object destructor)
		  *  # shutdown_cmd1 = XXXX
		  *  # shutdown_cmd2 = XXXX
		  *
		  *  \endcode
		  *
		  * Note that the `customInit` field, supported in MRPT <1.4.0 will be still parsed and obeyed, but since it has been superseded 
		  * by the new mechanism to establish set-up commands, it is no further documented here.
		  *
		  *  The next picture summarizes existing MRPT classes related to GPS / GNSS devices (CGPSInterface, CNTRIPEmitter, CGPS_NTRIP):
		  *
		  *  <div align=center> <img src="mrpt_gps_classes_usage.png"> </div>
		  *
		  * <b>VERSIONS HISTORY:</b>
		  * - 09/JUN/2006: First version (JLBC)
		  * - 04/JUN/2008: Added virtual methods for device-specific initialization commands.
		  * - 10/JUN/2008: Converted into CGenericSensor class (there are no inhirited classes anymore).
		  * - 07/DEC/2012: Added public static method to parse NMEA strings.
		  * - 17/JUN/2014: Added GGA feedback.
		  * - 01/FEB/2016: API changed for MTPT 1.4.0
		  *
		  *  \note Verbose debug info will be dumped to cout if the environment variable "MRPT_HWDRIVERS_VERBOSE" is set to "1", or if you call CGenericSensor::enableVerbose(true)
		  *  \note 
		  *  \note <b>[API changed in MRPT 1.4.0]</b> mrpt::hwdrivers::CGPSInterface API clean-up and made more generic so any stream can be used to parse GNSS messages, not only serial ports.
		  *
		  * \sa CGPS_NTRIP, CNTRIPEmitter, mrpt::obs::CObservationGPS 
		  * \ingroup mrpt_hwdrivers_grp
		  */
		class HWDRIVERS_IMPEXP CGPSInterface : public mrpt::utils::COutputLogger, public CGenericSensor
		{
			DEFINE_GENERIC_SENSOR(CGPSInterface)

		public:
			/** Read about parser selection in the documentation for CGPSInterface */
			enum PARSERS
			{
				NONE         = -2,
				AUTO         = -1,
				NMEA         = 0,
				NOVATEL_OEM6
			};

			CGPSInterface(); //!< Default ctor
			virtual ~CGPSInterface();  //!< Dtor

			void  doProcess(); // See docs in parent class

			bool  isGPS_connected(); //!< Returns true if communications work, i.e. if some message has been received.
			bool  isGPS_signalAcquired(); //!< Returns true if the last message from the GPS indicates that the signal from sats has been acquired.

			/** \name Set-up and configuration 
			  * @{ */
			void  setSerialPortName(const std::string &COM_port);  //!< Set the serial port to use (COM1, ttyUSB0, etc).
			std::string getSerialPortName() const;  //!< Get the serial port to use (COM1, ttyUSB0, etc).

			void  setParser(PARSERS parser);  //!< Select the parser for incomming data, among the options enumerated in \a CGPSInterface
			PARSERS getParser() const;

			//void setExternCOM( CSerialPort *outPort, mrpt::synch::CCriticalSection *csOutPort ); // Replaced by bindStream() in MRPT 1.4.0

			/** This enforces the use of a given user stream, instead of trying to open the serial port set in this class parameters.
			  * \param[in] csExternalStream If not NULL, read/write operations to the stream will be guarded by this critical section.
			  * The stream object is not deleted. It is the user responsibility to keep that object allocated during the entire life of this object.
			  * \note Call before CGenericSensor::initialize()
			  */
			void bindStream(mrpt::utils::CStream * external_stream, mrpt::synch::CCriticalSection *csOptionalExternalStream = NULL );

			bool useExternCOM() const { return m_data_stream_is_external; }
			bool useExternalStream() const { return m_data_stream_is_external; }

			void setSetupCommandsDelay(const double delay_secs);
			double getSetupCommandsDelay() const;

			void setSetupCommands(const std::vector<std::string> &cmds);
			const std::vector<std::string> & getSetupCommands() const;

			void setShutdownCommands(const std::vector<std::string> &cmds);
			const std::vector<std::string> & getShutdownCommands() const;

			void enableSetupCommandsAppendCRLF(const bool enable);
			bool isEnabledSetupCommandsAppendCRLF() const;

			void enableAppendMsgTypeToSensorLabel(bool enable) { m_sensorLabelAppendMsgType = enable; }

			/** Send a custom data block to the GNSS device right now. Can be used to change its behavior online as needed. 
			  \return false on communication error */
			bool sendCustomCommand(const void* data, const size_t datalen);
			/** @} */

			inline bool isAIMConfigured() { return m_topcon_AIMConfigured; }

			/** Parses one line of NMEA data from a GPS receiver, and writes the recognized fields (if any) into an observation object.
			  * Recognized frame types are those listed for the `NMEA` parser in the documentation of CGPSInterface
			  * \return true if some new data field has been correctly parsed and inserted into out_obs
			  */
			static bool parse_NMEA(const std::string &cmd_line, mrpt::obs::CObservationGPS &out_obs, const bool verbose=false);

			/** Gets the latest GGA command or an empty string if no newer GGA command was received since the last call to this method.
			  * \param[in] reset If set to true, will empty the GGA cache so next calls will return an empty string if no new frame is received.
			  */
			std::string getLastGGA(bool reset=true);

			typedef bool (CGPSInterface::*ptr_parser_t)(size_t &out_minimum_rx_buf_to_decide);

			/** @name Parser implementations: each method must try to parse the first bytes in the 
			  *  incoming buffer, and return false if the available data does not match the expected format, so we must skip 1 byte and try again.
			  * @{ */
			bool implement_parser_NMEA(size_t &out_minimum_rx_buf_to_decide);
			bool implement_parser_NOVATEL_OEM6(size_t &out_minimum_rx_buf_to_decide);
			/** @} */

		protected:
			/** Implements custom messages to be sent to the GPS unit just after connection and before normal use.
			  *  Returns false or raise an exception if something goes wrong. */
			bool OnConnectionEstablished();
			bool OnConnectionShutdown(); //!< Like OnConnectionEstablished() for sending optional shutdown commands

			bool legacy_topcon_setup_commands();

			mrpt::utils::CStream           *m_data_stream;    //!< Typically a CSerialPort created by this class, but may be set externally.
			mrpt::synch::CCriticalSection  *m_data_stream_cs;
			bool                            m_data_stream_is_external;

			poses::CPose3D  m_sensorPose;
			std::string     m_customInit;

			/** See the class documentation at the top for expected parameters */
			void  loadConfig_sensorSpecific(
				const mrpt::utils::CConfigFileBase &configSource,
				const std::string	  &iniSection );

			/** If not empty, will send a cmd "set,/par/pos/pd/port,...". Example value: "/dev/ser/b" */
			void setJAVAD_rtk_src_port( const std::string &s) { m_JAVAD_rtk_src_port = s; }

			/** Only used when "m_JAVAD_rtk_src_port" is not empty */
			void setJAVAD_rtk_src_baud(unsigned int baud) { m_JAVAD_rtk_src_baud = baud; }

			/** Only used when "m_JAVAD_rtk_src_port" is not empty: format of RTK corrections: "cmr", "rtcm", "rtcm3", etc. */
			void setJAVAD_rtk_format(const std::string &s) {m_JAVAD_rtk_format=s;}

			/** Set Advanced Input Mode for the primary port.
				This can be used to send RTK corrections to the device using the same port that it's used for the commands.
				The RTK correction stream must be re-packaged into a special frame with prefix ">>" */
			bool setJAVAD_AIM_mode();

			/** Unset Advanced Input Mode for the primary port and use it only as a command port. */
			bool unsetJAVAD_AIM_mode();

		private:
			mrpt::utils::circular_buffer<uint8_t> m_rx_buffer; //!< Auxiliary buffer for readings
			PARSERS      m_parser;
			std::string  m_raw_dump_file_prefix;
			std::string  m_COMname;
			int          m_COMbauds;
			bool         m_sensorLabelAppendMsgType;
			bool         m_GPS_comsWork;
			mrpt::system::TTimeStamp        m_last_timestamp;
			mrpt::utils::CFileOutputStream  m_raw_output_file;
			double                   m_custom_cmds_delay;
			bool                     m_custom_cmds_append_CRLF;
			std::vector<std::string> m_setup_cmds;
			std::vector<std::string> m_shutdown_cmds;

			/** \name Legacy support for TopCon RTK configuration
			  * @{ */
			std::string		m_JAVAD_rtk_src_port; 	//!< If not empty, will send a cmd "set,/par/pos/pd/port,...". Example value: "/dev/ser/b"
			unsigned int	m_JAVAD_rtk_src_baud; 	//!< Only used when "m_JAVAD_rtk_src_port" is not empty
			std::string		m_JAVAD_rtk_format; 	//!< Only used when "m_JAVAD_rtk_src_port" is not empty: format of RTK corrections: "cmr", "rtcm", "rtcm3", etc.

			bool            m_topcon_useAIMMode;           //!< Use this mode for receive RTK corrections from a external source through the primary port
			bool            m_topcon_AIMConfigured;        //!< Indicates if the AIM has been properly set up.
			double          m_topcon_data_period;          //!< The period in seconds which the data should be provided by the GPS
			void JAVAD_sendMessage(const char*str, bool waitForAnswer = true); //!< Private auxiliary method. Raises exception on error.
			/** @} */

			/** Returns true if the COM port is already open, or try to open it in other case.
			  * \return true if everything goes OK, or false if there are problems opening the port. */
			bool  tryToOpenTheCOM();

			void  parseBuffer(); //!< Process data in "m_buffer" to extract GPS messages, and remove them from the buffer.

			void  flushParsedMessagesNow();  //!< Queue out now the messages in \a m_just_parsed_messages, leaving it empty
			mrpt::obs::CObservationGPS  m_just_parsed_messages; //!< A private copy of the last received gps datum
			std::string   m_last_GGA; //!< Used in getLastGGA()
		}; // end class
	} // end namespace

	// Specializations MUST occur at the same namespace:
	namespace utils
	{
		template <>
		struct TEnumTypeFiller<hwdrivers::CGPSInterface::PARSERS>
		{
			typedef hwdrivers::CGPSInterface::PARSERS enum_t;
			static void fill(bimap<enum_t,std::string>  &m_map)
			{
				m_map.insert(hwdrivers::CGPSInterface::NONE,          "NONE");
				m_map.insert(hwdrivers::CGPSInterface::AUTO,          "AUTO");
				m_map.insert(hwdrivers::CGPSInterface::NMEA,          "NMEA");
				m_map.insert(hwdrivers::CGPSInterface::NOVATEL_OEM6,  "NOVATEL_OEM6");
			}
		};
	}
} // end namespace

#endif
