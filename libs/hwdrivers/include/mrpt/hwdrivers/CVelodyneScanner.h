/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/hwdrivers/CGenericSensor.h>
#include <mrpt/obs/CObservationVelodyneScan.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/typemeta/TEnumType.h>

namespace mrpt::hwdrivers
{
/** A C++ interface to Velodyne laser scanners (HDL-64, HDL-32, VLP-16), working
 * on Linux and Windows.
 * (Using this class requires WinPCap as a run-time dependency in Windows).
 * It can receive data from real devices via an Ethernet connection or parse a
 * WireShark PCAP file for offline processing.
 * The choice of online vs. offline operation is taken upon calling \a
 * initialize(): if a PCAP input file has been defined,
 * offline operation takes place and network is not listened for incomming
 * packets.
 *
 * Parsing dual return scans requires a VLP-16 with firmware version 3.0.23 or
 * newer. While converting the scan into a
 * point cloud in mrpt::obs::CObservationVelodyneScan you can select whether to
 * keep the strongest, the last or both laser returns.
 *
 * XML calibration files are not mandatory for VLP-16 and HDL-32, but they are
 * for HDL-64.
 *
 * <h2>Grabbing live data (as a user)</h2> <hr>
 *  - Use the application
 * [velodyne-view](http://www.mrpt.org/list-of-mrpt-apps/application-velodyne-view/)
 * to visualize the LIDAR output in real-time (optionally saving to a PCAP file)
 * or to playback a PCAP file.
 *  - Use
 * [rawlog-grabber](http://www.mrpt.org/list-of-mrpt-apps/application-rawlog-grabber/)
 * to record a dataset in MRPT's format together with any other set of sensors.
 * See example config file:
 * [MRPT\share\mrpt\config_files\rawlog-grabber\velodyne.ini](https://github.com/MRPT/mrpt/blob/master/share/mrpt/config_files/rawlog-grabber/velodyne.ini)
 *
 * <h2>Grabbing live data (programmatically)</h2> <hr>
 *  - See CGenericSensor for a general overview of the sequence of methods to
 * be called: loadConfig(), initialize(), doProcess().
 *  - Or use this class inside the application
 * [rawlog-grabber](http://www.mrpt.org/list-of-mrpt-apps/application-rawlog-grabber/).
 * See example config files:
 * [MRPT\share\mrpt\config_files\rawlog-grabber\velodyne.ini](https://github.com/MRPT/mrpt/blob/master/share/mrpt/config_files/rawlog-grabber/velodyne.ini)
 *
 * See the source code of the example application `[MRPT]/apps/velodyne-view`
 * ([velodyne-view web
 * page](http://www.mrpt.org/list-of-mrpt-apps/application-velodyne-view/)) for
 * more details.
 *
 * <h2>Playing back a PCAP file:</h2><hr>
 *  It is common to save Velodyne datasets as Wireshark's PCAP files.
 *  These files can be played back with tools like
 * [bittwist](http://bittwist.sourceforge.net/), which emit all UDP packets in
 * the PCAP log.
 *  Then, use this class to receive the packets as if they come from the real
 * sensor.
 *
 *  Alternatively, if MRPT is linked against libpcap, this class can directly
 * parse a PCAP file to simulate reading from a device offline.
 *  See method setPCAPInputFile() and config file parameter ``
 *
 *  To compile with PCAP support: In Debian/Ubuntu, install libpcap-dev. In
 * Windows, install WinPCap developer packages + the regular WinPCap driver.
 *
 *  <h2>Configuration and usage:</h2> <hr>
 * Data is returned as observations of type:
 *  - mrpt::obs::CObservationVelodyneScan for one or more "data packets" (refer
 * to Velodyne usage manual)
 *  - mrpt::obs::CObservationGPS for GPS (GPRMC) packets, if available via the
 * synchronization interface of the device.
 *  See those classes for documentation on their fields.
 *
 * Configuration includes setting the device IP (optional) and sensor model
 * (mandatory only if a calibration file is not provided).
 * These parameters can be set programmatically (see methods of this class), or
 * via a configuration file with CGenericSensor::loadConfig() (see example
 * config file section below).
 *
 * <h2>About timestamps:</h2><hr>
 *  Each gathered observation of type mrpt::obs::CObservationVelodyneScan is
 * populated with two timestamps, one for the local PC timestamp and,
 *  if available, another one for the GPS-stamped timestamp. Refer to the
 * observation docs for details.
 *
 * <h2>Format of parameters for loading from a .ini file</h2><hr>
 *  \code
 *  PARAMETERS IN THE ".INI"-LIKE CONFIGURATION STRINGS:
 * -------------------------------------------------------
 *   [supplied_section_name]
 *   # ---- Sensor description ----
 *   #calibration_file         = PUT_HERE_FULL_PATH_TO_CALIB_FILE.xml      //
 * Optional but recommended: put here your vendor-provided calibration file
 *   model                    = VLP16      // Can be any of: `VLP16`, `HDL32`,
 * `HDL64`  (It is used to load default calibration file. Parameter not required
 * if `calibration_file` is provided.
 *   #pos_packets_min_period  = 0.5        // (Default=0.5 seconds) Minimum
 * period to leave between reporting position packets. Used to decimate the
 * large number of packets of this type.
 *   # How long to wait, after loss of GPS signal, to report timestamps as "not
 * based on satellite time". 30 secs, with typical velodyne clock drifts, means
 * a ~1.7 ms typical drift.
 *   #pos_packets_timing_timeout = 30      // (Default=30 seconds)
 *   # ---- Online operation ----
 *
 *   # IP address of the device. UDP packets from other IPs will be ignored.
 * Leave commented or blank
 *   # if only one scanner is present (no IP filtering)
 *   #device_ip       = XXX.XXX.XXX.XXX
 *
 *   #rpm             = 600        // Sensor RPM (Default: unchanged). Requires
 * setting `device_ip`
 *   #return_type     = STRONGEST  // Any of: 'STRONGEST', 'LAST', 'DUAL'
 * (Default: unchanged). Requires setting `device_ip`
 *
 *   # ---- Offline operation ----
 *   # If uncommented, this class will read from the PCAP instead of connecting
 * and listeling
 *   # for online network packets.
 *   # pcap_input     = PUT_FULL_PATH_TO_PCAP_LOG_FILE.pcap
 *   # pcap_read_once = false   // Do not loop
 *   # pcap_read_fast = false    // fast forward skipping non-velodyne packets
 *   # pcap_read_full_scan_delay_ms = 100 // Used to simulate a reasonable
 * number of full scans / second
 *   # pcap_repeat_delay = 0.0   // seconds
 *
 *   # ---- Save to PCAP file ----
 *   # If uncommented, a PCAP file named
 * `[pcap_output_prefix]_[DATE_TIME].pcap` will be
 *   # written simultaneously to the normal operation of this class.
 *   # pcap_output     = velodyne_log
 *
 *   # 3D position of the sensor on the vehicle:
 *   pose_x     = 0    // 3D position (meters)
 *   pose_y     = 0
 *   pose_z     = 0
 *   pose_yaw   = 0    // 3D orientation (degrees)
 *   pose_pitch = 0
 *   pose_roll  = 0
 *
 *  \endcode
 *
 *
 * <h2>Copyright notice</h2><hr>
 * Portions of this class are based on code from velodyne ROS node in
 * https://github.com/ros-drivers/velodyne
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  License: Modified BSD Software License Agreement
 *
 * \note New in MRPT 1.4.0
 * \ingroup mrpt_hwdrivers_grp
 */
class CVelodyneScanner : public mrpt::hwdrivers::CGenericSensor
{
	DEFINE_GENERIC_SENSOR(CVelodyneScanner)
   public:
	/** Default: 2368. Change it if required. */
	static short int VELODYNE_DATA_UDP_PORT;
	/** Default: 8308. Change it if required. */
	static short int VELODYNE_POSITION_UDP_PORT;

	/** LIDAR model types */
	enum model_t
	{
		VLP16 = 1,
		HDL32 = 2,
		HDL64 = 3
	};

	/** LIDAR return type */
	enum return_type_t
	{
		UNCHANGED = 0,
		STRONGEST,
		LAST,
		DUAL
	};

	/** Hard-wired properties of LIDARs depending on the model */
	struct TModelProperties
	{
		double maxRange;
	};
	using model_properties_list_t = std::map<model_t, TModelProperties>;
	/** Access to default sets of parameters for Velodyne LIDARs */
	struct TModelPropertiesFactory
	{
		/** Singleton access */
		static const model_properties_list_t& get();
		/** Return human-readable string: "`VLP16`,`XXX`,..." */
		static std::string getListKnownModels();
	};

   protected:
	bool m_initialized;
	/** Default: "VLP16" */
	model_t m_model{CVelodyneScanner::VLP16};
	/** Default: 0.5 seconds */
	double m_pos_packets_min_period{0.5};
	/** Default: 30 seconds */
	double m_pos_packets_timing_timeout{30.0};
	/** Default: "" (no IP-based filtering) */
	std::string m_device_ip;
	/** Default: true Output whole frames and not data packets  */
	bool m_return_frames{true};
	/** Default: true Output PCAP Info msgs */
	bool m_pcap_verbose{true};
	/** Default: "" (do not operate from an offline file) */
	std::string m_pcap_input_file;
	/** Default: "" (do not dump to an offline file) */
	std::string m_pcap_output_file;
	mrpt::poses::CPose3D m_sensorPose;
	/** Device calibration file (supplied by vendor in an XML file) */
	mrpt::obs::VelodyneCalibration m_velodyne_calib;
	mrpt::system::TTimeStamp m_last_pos_packet_timestamp;

	// offline operation:
	/** opaque ptr: "pcap_t*" */
	void* m_pcap{nullptr};
	/** opaque ptr: "pcap_t*" */
	void* m_pcap_out{nullptr};
	/** opaque ptr: "pcap_dumper_t *" */
	void* m_pcap_dumper{nullptr};
	/** opaque ptr: bpf_program* */
	void* m_pcap_bpf_program{nullptr};
	bool m_pcap_file_empty{true};
	/** number of pkts read from the file so far (for debugging) */
	unsigned int m_pcap_read_count;
	/** Default: false */
	bool m_pcap_read_once{false};
	/** (Default: false) If false, will use m_pcap_read_full_scan_delay_ms */
	bool m_pcap_read_fast{false};
	/** (Default:100 ms) delay after each full scan read from a PCAP log */
	double m_pcap_read_full_scan_delay_ms{100};
	/** Default: 0 (in seconds) */
	double m_pcap_repeat_delay{0.0};

	/** See the class documentation at the top for expected parameters */
	void loadConfig_sensorSpecific(
		const mrpt::config::CConfigFileBase& configSource,
		const std::string& section) override;

   public:
	CVelodyneScanner();
	~CVelodyneScanner() override;

	/** @name Change configuration parameters; to be called BEFORE initialize();
	 * see above for the list of parameters and their meaning
	 * @{ */
	/** See supported model names in the general discussion docs for
	 * mrpt::hwdrivers::CVelodyneScanner */
	void setModelName(const model_t model) { m_model = model; }
	model_t getModelName() const { return m_model; }
	/** Set the minimum period between the generation of
	 * mrpt::obs::CObservationGPS observations from Velodyne Position RMC GPS
	 * packets */
	void setPosPacketsMinPeriod(double period_seconds)
	{
		m_pos_packets_min_period = period_seconds;
	}
	double getPosPacketsMinPeriod() const { return m_pos_packets_min_period; }
	/** Set how long to wait, after loss of GPS signal, to report timestamps as
	 * "not based on satellite time". 30 secs, with typical velodyne clock
	 * drifts, means a ~1.7 ms typical drift. */
	void setPosPacketsTimingTimeout(double timeout)
	{
		m_pos_packets_timing_timeout = timeout;
	}
	double getPosPacketsTimingTimeout() const
	{
		return m_pos_packets_timing_timeout;
	}

	/** UDP packets from other IPs will be ignored. Default: empty string, means
	 * do not filter by IP */
	void setDeviceIP(const std::string& ip) { m_device_ip = ip; }
	const std::string& getDeviceIP() const { return m_device_ip; }
	/** Enables/disables PCAP info messages to console (default: true) */
	void setPCAPVerbosity(const bool verbose) { m_pcap_verbose = verbose; }
	/** Enables reading from a PCAP file instead of live UDP packet listening */
	void setPCAPInputFile(const std::string& pcap_file)
	{
		m_pcap_input_file = pcap_file;
	}
	const std::string& getPCAPInputFile() const { return m_pcap_input_file; }
	/** Enables dumping to a PCAP file in parallel to returning regular MRPT
	 * objects. Default="": no pcap log. */
	void setPCAPOutputFile(const std::string& out_pcap_file)
	{
		m_pcap_output_file = out_pcap_file;
	}
	const std::string& getPCAPOutputFile() const { return m_pcap_output_file; }
	void setPCAPInputFileReadOnce(bool read_once)
	{
		m_pcap_read_once = read_once;
	}
	bool getPCAPInputFileReadOnce() const { return m_pcap_read_once; }
	const mrpt::obs::VelodyneCalibration& getCalibration() const
	{
		return m_velodyne_calib;
	}
	void setCalibration(const mrpt::obs::VelodyneCalibration& calib)
	{
		m_velodyne_calib = calib;
	}
	/** Returns false on error. \sa
	 * mrpt::obs::VelodyneCalibration::loadFromXMLFile() */
	bool loadCalibrationFile(const std::string& velodyne_xml_calib_file_path);

	/** Changes among STRONGEST, LAST, DUAL return types (via HTTP post
	 * interface).
	 * Can be called at any instant, before or after initialize().
	 * Requires setting a device IP address.
	 * \return false on error */
	bool setLidarReturnType(return_type_t ret_type);

	/** Changes Lidar RPM (valid range: 300-600) (via HTTP post interface).
	 * Can be called at any instant, before or after initialize().
	 * Requires setting a device IP address.
	 * \return false on error*/
	bool setLidarRPM(int rpm);

	/** Switches the LASER on/off (saves energy when not measuring) (via HTTP
	 * post interface).
	 * Can be called at any instant, before or after initialize().
	 * Requires setting a device IP address.
	 * \return false on error*/
	bool setLidarOnOff(bool on);

	/** Switches whole frame (points in a single revolution) on/off publication
	 * to data packet publication. When on, getNextObservation() will return
	 * true whenever a frame is avaliable, when off, getNextObservation() will
	 * return true whenever a data packet is avaliable. The default is on. When
	 * listening to data packets on a PCAP, pcap_read_fast is enforced.
	 */
	void setFramePublishing(bool on);

	/** @} */

	/** Polls the UDP port for incoming data packets. The user *must* call this
	 * method in a timely fashion to grab data as it it generated by the device.
	 *  The minimum call rate should be the expected number of data
	 * packets/second (!=scans/second). Checkout Velodyne user manual if in
	 * doubt.
	 *
	 * \param[out] outScan Upon return, an empty smart pointer will be found
	 * here if no new data was available. Otherwise, a valid scan.
	 * \param[out] outGPS  Upon return, an empty smart pointer will be found
	 * here if no new GPS data was available. Otherwise, a valid GPS reading.
	 * \return true if no error ocurred (even if there was no new observation).
	 * false if any communication error occurred.
	 */
	bool getNextObservation(
		mrpt::obs::CObservationVelodyneScan::Ptr& outScan,
		mrpt::obs::CObservationGPS::Ptr& outGPS);

	// See docs in parent class
	void doProcess() override;

	/** Tries to initialize the sensor driver, after setting all the parameters
	 * with a call to loadConfig.
	 * Velodyne specifics: this method sets up the UDP listening sockets, so
	 * all relevant params MUST BE SET BEFORE calling this.
	 *  \exception This method must throw an exception with a descriptive
	 * message if some critical error is found.
	 */
	void initialize() override;

	/** Close the UDP sockets set-up in \a initialize(). This is called
	 * automatically upon destruction */
	void close();

	/** Users normally would prefer calling \a getNextObservation() instead.
	 * This method polls the UDP data port and returns one Velodyne DATA packet
	 * (1206 bytes) and/or one POSITION packet. Refer to Velodyne users manual.
	 * Approximate timestamps (based on this computer clock) are returned for
	 * each kind of packets, or INVALID_TIMESTAMP if timeout ocurred waiting for
	 * a packet.
	 * \return true on all ok. false only for pcap reading EOF
	 */
	bool receivePackets(
		mrpt::system::TTimeStamp& data_pkt_timestamp,
		mrpt::obs::CObservationVelodyneScan::TVelodyneRawPacket& out_data_pkt,
		mrpt::system::TTimeStamp& pos_pkt_timestamp,
		mrpt::obs::CObservationVelodyneScan::TVelodynePositionPacket&
			out_pos_pkt);

   private:
	/** Handles for the UDP sockets, or INVALID_SOCKET (-1) */
	using platform_socket_t =
#ifdef _WIN32
#if MRPT_WORD_SIZE == 64
		uint64_t
#else
		uint32_t
#endif
#else
		int
#endif
		;

	platform_socket_t m_hDataSock, m_hPositionSock;

	static mrpt::system::TTimeStamp internal_receive_UDP_packet(
		platform_socket_t hSocket, uint8_t* out_buffer,
		const size_t expected_packet_size,
		const std::string& filter_only_from_IP);

	bool internal_read_PCAP_packet(
		mrpt::system::TTimeStamp& data_pkt_time, uint8_t* out_data_buffer,
		mrpt::system::TTimeStamp& pos_pkt_time, uint8_t* out_pos_buffer);

	/** In progress RX scan */
	mrpt::obs::CObservationVelodyneScan::Ptr m_rx_scan;

	mrpt::obs::gnss::Message_NMEA_RMC m_last_gps_rmc;
	mrpt::system::TTimeStamp m_last_gps_rmc_age;
	int m_lidar_rpm{0};
	return_type_t m_lidar_return{UNCHANGED};

	bool internal_send_http_post(const std::string& post_data);

};  // end of class
}  // namespace mrpt::hwdrivers
MRPT_ENUM_TYPE_BEGIN(mrpt::hwdrivers::CVelodyneScanner::model_t)
using namespace mrpt::hwdrivers;
MRPT_FILL_ENUM_MEMBER(CVelodyneScanner, VLP16);
MRPT_FILL_ENUM_MEMBER(CVelodyneScanner, HDL32);
MRPT_FILL_ENUM_MEMBER(CVelodyneScanner, HDL64);
MRPT_ENUM_TYPE_END()

MRPT_ENUM_TYPE_BEGIN(mrpt::hwdrivers::CVelodyneScanner::return_type_t)
using namespace mrpt::hwdrivers;
MRPT_FILL_ENUM_MEMBER(CVelodyneScanner, UNCHANGED);
MRPT_FILL_ENUM_MEMBER(CVelodyneScanner, STRONGEST);
MRPT_FILL_ENUM_MEMBER(CVelodyneScanner, LAST);
MRPT_FILL_ENUM_MEMBER(CVelodyneScanner, DUAL);
MRPT_ENUM_TYPE_END()
