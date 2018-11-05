/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "hwdrivers-precomp.h"  // Precompiled headers

#include <mrpt/hwdrivers/CVelodyneScanner.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/comms/net_utils.h>
#include <mrpt/hwdrivers/CGPSInterface.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/datetime.h>  // timeDifference
#include <mrpt/core/reverse_bytes.h>
#include <thread>

// socket's hdrs:
#ifdef _WIN32
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#if defined(_WIN32_WINNT) && (_WIN32_WINNT < 0x600)
#undef _WIN32_WINNT
#define _WIN32_WINNT 0x600  // Minimum: Windows Vista (required to pollfd)
#endif

#include <winsock2.h>
using socklen_t = int;

#if defined(_MSC_VER)
#pragma comment(lib, "WS2_32.LIB")
#endif

#else
#define INVALID_SOCKET (-1)
#include <sys/time.h>  // gettimeofday()
#include <sys/socket.h>
#include <unistd.h>
#include <fcntl.h>
#include <cerrno>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#endif

#if MRPT_HAS_LIBPCAP
#include <pcap.h>
#if !defined(PCAP_NETMASK_UNKNOWN)  // for older pcap versions
#define PCAP_NETMASK_UNKNOWN 0xffffffff
#endif
#endif

using namespace mrpt;
using namespace mrpt::hwdrivers;
using namespace mrpt::obs;
using namespace std;

IMPLEMENTS_GENERIC_SENSOR(CVelodyneScanner, mrpt::hwdrivers)

short int CVelodyneScanner::VELODYNE_DATA_UDP_PORT = 2368;
short int CVelodyneScanner::VELODYNE_POSITION_UDP_PORT = 8308;

const CVelodyneScanner::model_properties_list_t&
	CVelodyneScanner::TModelPropertiesFactory::get()
{
	static CVelodyneScanner::model_properties_list_t modelProperties;
	static bool init = false;
	if (!init)
	{
		init = true;
		modelProperties[CVelodyneScanner::VLP16].maxRange = 120.0;
		modelProperties[CVelodyneScanner::HDL32].maxRange = 70.0;
		modelProperties[CVelodyneScanner::HDL64].maxRange = 120.0;
	}
	return modelProperties;
}
std::string CVelodyneScanner::TModelPropertiesFactory::getListKnownModels()
{
	const CVelodyneScanner::model_properties_list_t& lst =
		CVelodyneScanner::TModelPropertiesFactory::get();
	std::string s;
	for (auto it : lst)
		s += mrpt::format(
			"`%s`,",
			mrpt::typemeta::TEnumType<CVelodyneScanner::model_t>::value2name(
				it.first)
				.c_str());
	return s;
}

CVelodyneScanner::CVelodyneScanner()
	: m_device_ip(""),

	  m_last_pos_packet_timestamp(INVALID_TIMESTAMP),

	  m_hDataSock(INVALID_SOCKET),
	  m_hPositionSock(INVALID_SOCKET),
	  m_last_gps_rmc_age(INVALID_TIMESTAMP)

{
	m_sensorLabel = "Velodyne";

#if MRPT_HAS_LIBPCAP
	m_pcap_bpf_program = new bpf_program[1];
#endif

#ifdef _WIN32
	// Init the WinSock Library:
	WORD wVersionRequested = MAKEWORD(2, 0);
	WSADATA wsaData;
	if (WSAStartup(wVersionRequested, &wsaData))
		THROW_EXCEPTION("Error calling WSAStartup");
#endif
}

CVelodyneScanner::~CVelodyneScanner()
{
	this->close();
#ifdef _WIN32
	WSACleanup();
#endif

#if MRPT_HAS_LIBPCAP
	delete[] reinterpret_cast<bpf_program*>(m_pcap_bpf_program);
	m_pcap_bpf_program = nullptr;
#endif
}

bool CVelodyneScanner::loadCalibrationFile(
	const std::string& velodyne_xml_calib_file_path)
{
	return m_velodyne_calib.loadFromXMLFile(velodyne_xml_calib_file_path);
}

void CVelodyneScanner::loadConfig_sensorSpecific(
	const mrpt::config::CConfigFileBase& cfg, const std::string& sect)
{
	MRPT_START

	cfg.read_enum<CVelodyneScanner::model_t>(sect, "model", m_model);
	MRPT_LOAD_HERE_CONFIG_VAR(device_ip, string, m_device_ip, cfg, sect);
	MRPT_LOAD_HERE_CONFIG_VAR(pcap_input, string, m_pcap_input_file, cfg, sect);
	MRPT_LOAD_HERE_CONFIG_VAR(
		pcap_output, string, m_pcap_output_file, cfg, sect);
	MRPT_LOAD_HERE_CONFIG_VAR(
		pcap_read_once, bool, m_pcap_read_once, cfg, sect);
	MRPT_LOAD_HERE_CONFIG_VAR(
		pcap_read_fast, bool, m_pcap_read_fast, cfg, sect);
	MRPT_LOAD_HERE_CONFIG_VAR(
		pcap_read_full_scan_delay_ms, double, m_pcap_read_full_scan_delay_ms,
		cfg, sect);
	MRPT_LOAD_HERE_CONFIG_VAR(
		pcap_repeat_delay, double, m_pcap_repeat_delay, cfg, sect);
	MRPT_LOAD_HERE_CONFIG_VAR(
		pos_packets_timing_timeout, double, m_pos_packets_timing_timeout, cfg,
		sect);
	MRPT_LOAD_HERE_CONFIG_VAR(
		pos_packets_min_period, double, m_pos_packets_min_period, cfg, sect);

	using mrpt::DEG2RAD;
	m_sensorPose = mrpt::poses::CPose3D(
		cfg.read_float(sect, "pose_x", 0), cfg.read_float(sect, "pose_y", 0),
		cfg.read_float(sect, "pose_z", 0),
		DEG2RAD(cfg.read_float(sect, "pose_yaw", 0)),
		DEG2RAD(cfg.read_float(sect, "pose_pitch", 0)),
		DEG2RAD(cfg.read_float(sect, "pose_roll", 0)));

	std::string calibration_file;
	MRPT_LOAD_CONFIG_VAR(calibration_file, string, cfg, sect);
	if (!calibration_file.empty()) this->loadCalibrationFile(calibration_file);

	// Check validity:
	const model_properties_list_t& lstModels = TModelPropertiesFactory::get();
	if (lstModels.find(m_model) == lstModels.end())
	{
		THROW_EXCEPTION(mrpt::format(
			"Unrecognized `model` parameter: `%u` . Known values are: %s",
			static_cast<unsigned int>(m_model),
			TModelPropertiesFactory::getListKnownModels().c_str()))
	}

	// Optional HTTP-based settings:
	MRPT_LOAD_HERE_CONFIG_VAR(rpm, int, m_lidar_rpm, cfg, sect);
	m_lidar_return =
		cfg.read_enum<return_type_t>(sect, "return_type", m_lidar_return);

	MRPT_END
}

bool CVelodyneScanner::getNextObservation(
	mrpt::obs::CObservationVelodyneScan::Ptr& outScan,
	mrpt::obs::CObservationGPS::Ptr& outGPS)
{
	try
	{
		ASSERTMSG_(m_initialized, "initialize() has not been called yet!");

		// Init with empty smart pointers:
		outScan = mrpt::obs::CObservationVelodyneScan::Ptr();
		outGPS = mrpt::obs::CObservationGPS::Ptr();

		// Try to get data & pos packets:
		mrpt::obs::CObservationVelodyneScan::TVelodyneRawPacket rx_pkt;
		mrpt::obs::CObservationVelodyneScan::TVelodynePositionPacket rx_pos_pkt;
		mrpt::system::TTimeStamp data_pkt_timestamp, pos_pkt_timestamp;
		bool rx_all_ok = this->receivePackets(
			data_pkt_timestamp, rx_pkt, pos_pkt_timestamp, rx_pos_pkt);

		if (!rx_all_ok)
		{
			// PCAP EOF:
			return false;
		}

		if (pos_pkt_timestamp != INVALID_TIMESTAMP)
		{
			mrpt::obs::CObservationGPS::Ptr gps_obs =
				mrpt::make_aligned_shared<mrpt::obs::CObservationGPS>();
			gps_obs->sensorLabel = this->m_sensorLabel + std::string("_GPS");
			gps_obs->sensorPose = m_sensorPose;

			gps_obs->originalReceivedTimestamp = pos_pkt_timestamp;

			bool parsed_ok = CGPSInterface::parse_NMEA(
				std::string(rx_pos_pkt.NMEA_GPRMC), *gps_obs);
			const mrpt::obs::gnss::Message_NMEA_RMC* msg_rmc =
				gps_obs->getMsgByClassPtr<mrpt::obs::gnss::Message_NMEA_RMC>();
			if (!parsed_ok || !msg_rmc || msg_rmc->fields.validity_char != 'A')
			{
				gps_obs->has_satellite_timestamp = false;
				gps_obs->timestamp = pos_pkt_timestamp;
			}
			else
			{
				// We have live GPS signal and a recent RMC frame:
				m_last_gps_rmc_age = pos_pkt_timestamp;
				m_last_gps_rmc = *msg_rmc;
			}
			outGPS = gps_obs;  // save in output object
		}

		if (data_pkt_timestamp != INVALID_TIMESTAMP)
		{
			m_state = ssWorking;

			// Break into a new observation object when the azimuth passes
			// 360->0 deg:
			const uint16_t rx_pkt_start_angle = rx_pkt.blocks[0].rotation;
			// const uint16_t rx_pkt_end_angle   =
			// rx_pkt.blocks[CObservationVelodyneScan::BLOCKS_PER_PACKET-1].rotation;

			// Return the observation as done when a complete 360 deg scan is
			// ready:
			if (m_rx_scan && !m_rx_scan->scan_packets.empty())
			{
				if ((rx_pkt_start_angle <
					 m_rx_scan->scan_packets.rbegin()->blocks[0].rotation) ||
					!m_return_frames)
				{
					outScan = m_rx_scan;
					m_rx_scan.reset();

					if (m_pcap)
					{
						// Keep the reader from blowing through the file.
						if (!m_pcap_read_fast)
							std::this_thread::sleep_for(
								std::chrono::duration<double, std::milli>(
									m_pcap_read_full_scan_delay_ms));
					}
				}
			}

			// Create smart ptr to new in-progress observation:
			if (!m_rx_scan)
			{
				m_rx_scan = mrpt::make_aligned_shared<
					mrpt::obs::CObservationVelodyneScan>();
				m_rx_scan->sensorLabel =
					this->m_sensorLabel + std::string("_SCAN");
				m_rx_scan->sensorPose = m_sensorPose;
				m_rx_scan->calibration =
					m_velodyne_calib;  // Embed a copy of the calibration info

				{
					const model_properties_list_t& lstModels =
						TModelPropertiesFactory::get();
					auto it = lstModels.find(this->m_model);
					if (it != lstModels.end())
					{  // Model params:
						m_rx_scan->maxRange = it->second.maxRange;
					}
					else  // default params:
					{
						m_rx_scan->maxRange = 120.0;
					}
				}
			}

			// For the first packet, set timestamp:
			if (m_rx_scan->scan_packets.empty())
			{
				m_rx_scan->originalReceivedTimestamp = data_pkt_timestamp;
				// Using GPS, if available:
				if (m_last_gps_rmc.fields.validity_char == 'A' &&
					mrpt::system::timeDifference(
						m_last_gps_rmc_age, data_pkt_timestamp) <
						m_pos_packets_timing_timeout)
				{
					// Each Velodyne data packet has a timestamp field,
					// with the number of us since the top of the current HOUR:
					// take the date and time from the GPS, then modify minutes
					// and seconds from data pkt:
					const mrpt::system::TTimeStamp gps_tim =
						m_last_gps_rmc.fields.UTCTime.getAsTimestamp(
							m_last_gps_rmc.getDateAsTimestamp());

					mrpt::system::TTimeParts tim_parts;
					mrpt::system::timestampToParts(gps_tim, tim_parts);
					tim_parts.minute =
						rx_pkt.gps_timestamp /*us from top of hour*/ /
						60000000ul;
					tim_parts.second =
						(rx_pkt.gps_timestamp /*us from top of hour*/ %
						 60000000ul) *
						1e-6;

					const mrpt::system::TTimeStamp data_pkt_tim =
						mrpt::system::buildTimestampFromParts(tim_parts);

					m_rx_scan->timestamp = data_pkt_tim;
					m_rx_scan->has_satellite_timestamp = true;
				}
				else
				{
					m_rx_scan->has_satellite_timestamp = false;
					m_rx_scan->timestamp = data_pkt_timestamp;
				}
			}

			// Accumulate pkts in the observation object:
			m_rx_scan->scan_packets.push_back(rx_pkt);
		}

		return true;
	}
	catch (exception& e)
	{
		cerr << "[CVelodyneScanner::getObservation] Returning false due to "
				"exception: "
			 << endl;
		cerr << e.what() << endl;
		return false;
	}
}

void CVelodyneScanner::doProcess()
{
	CObservationVelodyneScan::Ptr obs;
	CObservationGPS::Ptr obs_gps;

	if (getNextObservation(obs, obs_gps))
	{
		m_state = ssWorking;
		if (obs) appendObservation(obs);
		if (obs_gps) appendObservation(obs_gps);
	}
	else
	{
		m_state = ssError;
		cerr << "ERROR receiving data from Velodyne devic!" << endl;
	}
}

/** Tries to initialize the sensor, after setting all the parameters with a call
 * to loadConfig.
 *  \exception This method must throw an exception with a descriptive message if
 * some critical error is found. */
void CVelodyneScanner::initialize()
{
	this->close();

	// (0) Preparation:
	// --------------------------------
	// Make sure we have calibration data:
	if (m_velodyne_calib.empty())
	{
		// Try to load default data:
		m_velodyne_calib = VelodyneCalibration::LoadDefaultCalibration(
			mrpt::typemeta::TEnumType<CVelodyneScanner::model_t>::value2name(
				m_model));
		if (m_velodyne_calib.empty())
			THROW_EXCEPTION(
				"Could not find default calibration data for the given LIDAR "
				"`model` name. Please, specify a valid `model` or load a valid "
				"XML configuration file first.");
	}

	// online vs off line operation??
	// -------------------------------
	if (m_pcap_input_file.empty())
	{  // Online

		if (m_lidar_rpm > 0)
		{
			if (!setLidarRPM(m_lidar_rpm))
				THROW_EXCEPTION("Error in setLidarRPM();");
		}
		if (m_lidar_return != UNCHANGED)
		{
			if (!setLidarReturnType(m_lidar_return))
				THROW_EXCEPTION("Error in setLidarReturnType();");
		}

		// (1) Create LIDAR DATA socket
		// --------------------------------
		if (INVALID_SOCKET == (m_hDataSock = socket(PF_INET, SOCK_DGRAM, 0)))
			THROW_EXCEPTION(format(
				"Error creating UDP socket:\n%s",
				mrpt::comms::net::getLastSocketErrorStr().c_str()));

		struct sockaddr_in bindAddr;
		memset(&bindAddr, 0, sizeof(bindAddr));
		bindAddr.sin_family = AF_INET;
		bindAddr.sin_port = htons(VELODYNE_DATA_UDP_PORT);
		bindAddr.sin_addr.s_addr = INADDR_ANY;

		if (int(INVALID_SOCKET) ==
			::bind(
				m_hDataSock, (struct sockaddr*)(&bindAddr), sizeof(sockaddr)))
			THROW_EXCEPTION(mrpt::comms::net::getLastSocketErrorStr());

#ifdef _WIN32
		unsigned long non_block_mode = 1;
		if (ioctlsocket(m_hDataSock, FIONBIO, &non_block_mode))
			THROW_EXCEPTION(
				"Error entering non-blocking mode with ioctlsocket().");
#else
		int oldflags = fcntl(m_hDataSock, F_GETFL, 0);
		if (oldflags == -1)
			THROW_EXCEPTION("Error retrieving fcntl() of socket.");
		oldflags |= O_NONBLOCK | FASYNC;
		if (-1 == fcntl(m_hDataSock, F_SETFL, oldflags))
			THROW_EXCEPTION("Error entering non-blocking mode with fcntl();");
#endif

		// (2) Create LIDAR POSITION socket
		// --------------------------------
		if (INVALID_SOCKET ==
			(m_hPositionSock = socket(PF_INET, SOCK_DGRAM, 0)))
			THROW_EXCEPTION(format(
				"Error creating UDP socket:\n%s",
				mrpt::comms::net::getLastSocketErrorStr().c_str()));

		bindAddr.sin_port = htons(VELODYNE_POSITION_UDP_PORT);

		if (int(INVALID_SOCKET) == ::bind(
									   m_hPositionSock,
									   (struct sockaddr*)(&bindAddr),
									   sizeof(sockaddr)))
			THROW_EXCEPTION(mrpt::comms::net::getLastSocketErrorStr());

#ifdef _WIN32
		if (ioctlsocket(m_hPositionSock, FIONBIO, &non_block_mode))
			THROW_EXCEPTION(
				"Error entering non-blocking mode with ioctlsocket().");
#else
		oldflags = fcntl(m_hPositionSock, F_GETFL, 0);
		if (oldflags == -1)
			THROW_EXCEPTION("Error retrieving fcntl() of socket.");
		oldflags |= O_NONBLOCK | FASYNC;
		if (-1 == fcntl(m_hPositionSock, F_SETFL, oldflags))
			THROW_EXCEPTION("Error entering non-blocking mode with fcntl();");
#endif
	}
	else
	{  // Offline:
#if MRPT_HAS_LIBPCAP
		char errbuf[PCAP_ERRBUF_SIZE];

		if (m_pcap_verbose)
			printf(
				"\n[CVelodyneScanner] Opening PCAP file \"%s\"\n",
				m_pcap_input_file.c_str());
		if ((m_pcap = pcap_open_offline(m_pcap_input_file.c_str(), errbuf)) ==
			nullptr)
		{
			THROW_EXCEPTION_FMT("Error opening PCAP file: '%s'", errbuf);
		}

		// Build PCAP filter:
		{
			std::string filter_str = mrpt::format(
				"(udp dst port %d || udp dst port %d)",
				static_cast<int>(CVelodyneScanner::VELODYNE_DATA_UDP_PORT),
				static_cast<int>(CVelodyneScanner::VELODYNE_POSITION_UDP_PORT));
			if (!m_device_ip.empty())
				filter_str += "&& src host " + m_device_ip;

			static std::string sMsgError =
				"[CVelodyneScanner] Error calling pcap_compile: ";  // This is
			// to avoid
			// the
			// ill-formed
			// signature
			// of
			// pcap_error()
			// accepting
			// "char*",
			// not
			// "const
			// char*"...
			// sigh
			if (pcap_compile(
					reinterpret_cast<pcap_t*>(m_pcap),
					reinterpret_cast<bpf_program*>(m_pcap_bpf_program),
					filter_str.c_str(), 1, PCAP_NETMASK_UNKNOWN) < 0)
				pcap_perror(reinterpret_cast<pcap_t*>(m_pcap), &sMsgError[0]);
		}

		m_pcap_file_empty = true;
		m_pcap_read_count = 0;

#else
		THROW_EXCEPTION(
			"Velodyne: Reading from PCAP requires building MRPT with libpcap "
			"support!");
#endif
	}

	m_last_pos_packet_timestamp = INVALID_TIMESTAMP;
	m_last_gps_rmc_age = INVALID_TIMESTAMP;
	m_state = ssInitializing;

	m_initialized = true;
}

void CVelodyneScanner::close()
{
	if (m_hDataSock != INVALID_SOCKET)
	{
		shutdown(m_hDataSock, 2);  // SD_BOTH  );
#ifdef _WIN32
		closesocket(m_hDataSock);
#else
		::close(m_hDataSock);
#endif
		m_hDataSock = INVALID_SOCKET;
	}

	if (m_hPositionSock != INVALID_SOCKET)
	{
		shutdown(m_hPositionSock, 2);  // SD_BOTH  );
#ifdef _WIN32
		closesocket(m_hPositionSock);
#else
		::close(m_hPositionSock);
#endif
		m_hPositionSock = INVALID_SOCKET;
	}
#if MRPT_HAS_LIBPCAP
	if (m_pcap)
	{
		pcap_close(reinterpret_cast<pcap_t*>(m_pcap));
		m_pcap = nullptr;
	}
	if (m_pcap_dumper)
	{
		pcap_dump_close(reinterpret_cast<pcap_dumper_t*>(m_pcap_dumper));
		m_pcap_dumper = nullptr;
	}
	if (m_pcap_out)
	{
		pcap_close(reinterpret_cast<pcap_t*>(m_pcap_out));
		m_pcap_out = nullptr;
	}
#endif
	m_initialized = false;
}

// Fixed Ethernet headers for PCAP capture --------
#if MRPT_HAS_LIBPCAP
const uint16_t LidarPacketHeader[21] = {0xffff, 0xffff, 0xffff, 0x7660, 0x0088,
										0x0000, 0x0008, 0x0045, 0xd204, 0x0000,
										0x0040, 0x11ff, 0xaab4, 0xa8c0, 0xc801,
										0xffff,  // checksum 0xa9b4 //source ip
										// 0xa8c0, 0xc801 is
										// 192.168.1.200
										0xffff, 0x4009, 0x4009, 0xbe04, 0x0000};
const uint16_t PositionPacketHeader[21] = {
	0xffff, 0xffff, 0xffff, 0x7660, 0x0088, 0x0000, 0x0008, 0x0045, 0xd204,
	0x0000, 0x0040, 0x11ff, 0xaab4, 0xa8c0, 0xc801, 0xffff,  // checksum 0xa9b4
	// //source ip
	// 0xa8c0, 0xc801
	// is 192.168.1.200
	0xffff, 0x7420, 0x7420, 0x0802, 0x0000};
#endif

#if defined(_WIN32) && MRPT_HAS_LIBPCAP
int gettimeofday(struct timeval* tp, void*)
{
	FILETIME ft;
	::GetSystemTimeAsFileTime(&ft);
	long long t =
		(static_cast<long long>(ft.dwHighDateTime) << 32) | ft.dwLowDateTime;
	t -= 116444736000000000LL;
	t /= 10;  // microseconds
	tp->tv_sec = static_cast<long>(t / 1000000UL);
	tp->tv_usec = static_cast<long>(t % 1000000UL);
	return 0;
}
#endif

bool CVelodyneScanner::receivePackets(
	mrpt::system::TTimeStamp& data_pkt_timestamp,
	mrpt::obs::CObservationVelodyneScan::TVelodyneRawPacket& out_data_pkt,
	mrpt::system::TTimeStamp& pos_pkt_timestamp,
	mrpt::obs::CObservationVelodyneScan::TVelodynePositionPacket& out_pos_pkt)
{
	static_assert(
		sizeof(mrpt::obs::CObservationVelodyneScan::TVelodynePositionPacket) ==
			CObservationVelodyneScan::POS_PACKET_SIZE,
		"Velodyne pos packet: wrong size!");
	static_assert(
		sizeof(mrpt::obs::CObservationVelodyneScan::TVelodyneRawPacket) ==
			CObservationVelodyneScan::PACKET_SIZE,
		"Velodyne raw packet: wrong size!");

	bool ret = true;  // all ok
	if (m_pcap)
	{
		ret = internal_read_PCAP_packet(
			data_pkt_timestamp, (uint8_t*)&out_data_pkt, pos_pkt_timestamp,
			(uint8_t*)&out_pos_pkt);
	}
	else
	{
		data_pkt_timestamp = internal_receive_UDP_packet(
			m_hDataSock, (uint8_t*)&out_data_pkt,
			CObservationVelodyneScan::PACKET_SIZE, m_device_ip);
		pos_pkt_timestamp = internal_receive_UDP_packet(
			m_hPositionSock, (uint8_t*)&out_pos_pkt,
			CObservationVelodyneScan::POS_PACKET_SIZE, m_device_ip);
	}

// Optional PCAP dump:
#if MRPT_HAS_LIBPCAP
	// Save to PCAP file?
	if (!m_pcap_output_file.empty() &&
		!m_pcap_out)  // 1st time: create output file
	{
#if MRPT_HAS_LIBPCAP
		char errbuf[PCAP_ERRBUF_SIZE];

		mrpt::system::TTimeParts parts;
		mrpt::system::timestampToParts(mrpt::system::now(), parts, true);
		string sFilePostfix = "_";
		sFilePostfix += format(
			"%04u-%02u-%02u_%02uh%02um%02us", (unsigned int)parts.year,
			(unsigned int)parts.month, (unsigned int)parts.day,
			(unsigned int)parts.hour, (unsigned int)parts.minute,
			(unsigned int)parts.second);
		const string sFileName =
			m_pcap_output_file +
			mrpt::system::fileNameStripInvalidChars(sFilePostfix) +
			string(".pcap");

		m_pcap_out = pcap_open_dead(DLT_EN10MB, 65535);
		ASSERTMSG_(
			m_pcap_out != nullptr, "Error creating PCAP live capture handle");

		printf(
			"\n[CVelodyneScanner] Writing to PCAP file \"%s\"\n",
			sFileName.c_str());
		if ((m_pcap_dumper = pcap_dump_open(
				 reinterpret_cast<pcap_t*>(m_pcap_out), sFileName.c_str())) ==
			nullptr)
		{
			THROW_EXCEPTION_FMT(
				"Error creating PCAP live dumper: '%s'", errbuf);
		}
#else
		THROW_EXCEPTION(
			"Velodyne: Writing PCAP files requires building MRPT with libpcap "
			"support!");
#endif
	}

	if (m_pcap_out && m_pcap_dumper &&
		(data_pkt_timestamp != INVALID_TIMESTAMP ||
		 pos_pkt_timestamp != INVALID_TIMESTAMP))
	{
		struct pcap_pkthdr header;
		struct timeval currentTime;
		gettimeofday(&currentTime, nullptr);
		std::vector<unsigned char> packetBuffer;

		// Data pkt:
		if (data_pkt_timestamp != INVALID_TIMESTAMP)
		{
			header.caplen = header.len =
				CObservationVelodyneScan::PACKET_SIZE + 42;
			packetBuffer.resize(header.caplen);
			header.ts = currentTime;

			memcpy(&(packetBuffer[0]), LidarPacketHeader, 42);
			memcpy(
				&(packetBuffer[0]) + 42, (uint8_t*)&out_data_pkt,
				CObservationVelodyneScan::PACKET_SIZE);
			pcap_dump(
				(u_char*)this->m_pcap_dumper, &header, &(packetBuffer[0]));
		}
		// Pos pkt:
		if (pos_pkt_timestamp != INVALID_TIMESTAMP)
		{
			header.caplen = header.len =
				CObservationVelodyneScan::POS_PACKET_SIZE + 42;
			packetBuffer.resize(header.caplen);
			header.ts = currentTime;

			memcpy(&(packetBuffer[0]), PositionPacketHeader, 42);
			memcpy(
				&(packetBuffer[0]) + 42, (uint8_t*)&out_pos_pkt,
				CObservationVelodyneScan::POS_PACKET_SIZE);
			pcap_dump(
				(u_char*)this->m_pcap_dumper, &header, &(packetBuffer[0]));
		}
	}
#endif

// Convert from Velodyne's standard little-endian ordering to host byte
// ordering:
// (done AFTER saving the pckg as is to pcap above)
#if MRPT_IS_BIG_ENDIAN
	if (data_pkt_timestamp != INVALID_TIMESTAMP)
	{
		mrpt::utils::reverseBytesInPlace(out_data_pkt.gps_timestamp);
		for (int i = 0; i < CObservationVelodyneScan::BLOCKS_PER_PACKET; i++)
		{
			mrpt::utils::reverseBytesInPlace(out_data_pkt.blocks[i].header);
			mrpt::utils::reverseBytesInPlace(out_data_pkt.blocks[i].rotation);
			for (int k = 0; k < CObservationVelodyneScan::SCANS_PER_BLOCK; k++)
			{
				mrpt::utils::reverseBytesInPlace(
					out_data_pkt.blocks[i].laser_returns[k].distance);
			}
		}
	}
	if (pos_pkt_timestamp != INVALID_TIMESTAMP)
	{
		mrpt::utils::reverseBytesInPlace(out_pos_pkt.gps_timestamp);
		mrpt::utils::reverseBytesInPlace(out_pos_pkt.unused2);
	}
#endif

	// Position packet decimation:
	if (pos_pkt_timestamp != INVALID_TIMESTAMP)
	{
		if ((m_last_pos_packet_timestamp != INVALID_TIMESTAMP) &&
			mrpt::system::timeDifference(
				m_last_pos_packet_timestamp, pos_pkt_timestamp) <
				m_pos_packets_min_period)
		{
			// Ignore this packet
			pos_pkt_timestamp = INVALID_TIMESTAMP;
		}
		else
		{
			// Reset time watch:
			m_last_pos_packet_timestamp = pos_pkt_timestamp;
		}
	}

	return ret;
}

// static method:
mrpt::system::TTimeStamp CVelodyneScanner::internal_receive_UDP_packet(
	platform_socket_t hSocket, uint8_t* out_buffer,
	const size_t expected_packet_size, const std::string& filter_only_from_IP)
{
	if (hSocket == INVALID_SOCKET)
		THROW_EXCEPTION(
			"Error: UDP socket is not open yet! Have you called initialize() "
			"first?");

	unsigned long devip_addr = 0;
	if (!filter_only_from_IP.empty())
		devip_addr = inet_addr(filter_only_from_IP.c_str());

	const mrpt::system::TTimeStamp time1 = mrpt::system::now();

	struct pollfd fds[1];
	fds[0].fd = hSocket;
	fds[0].events = POLLIN;
	static const int POLL_TIMEOUT = 1;  // (ms)

	sockaddr_in sender_address;
	socklen_t sender_address_len = sizeof(sender_address);

	while (true)
	{
		// Unfortunately, the Linux kernel recvfrom() implementation
		// uses a non-interruptible std::this_thread::sleep_for(ms) when waiting
		// for data,
		// which would cause this method to hang if the device is not
		// providing data.  We poll() the device first to make sure
		// the recvfrom() will not block.
		//
		// Note, however, that there is a known Linux kernel bug:
		//
		//   Under Linux, select() may report a socket file descriptor
		//   as "ready for reading", while nevertheless a subsequent
		//   read blocks.  This could for example happen when data has
		//   arrived but upon examination has wrong checksum and is
		//   discarded.  There may be other circumstances in which a
		//   file descriptor is spuriously reported as ready.  Thus it
		//   may be safer to use O_NONBLOCK on sockets that should not
		//   block.

		// poll() until input available
		do
		{
			int retval =
#if !defined(_WIN32)
				poll
#else
				WSAPoll
#endif
				(fds, 1, POLL_TIMEOUT);
			if (retval < 0)  // poll() error?
			{
				if (errno != EINTR)
					THROW_EXCEPTION(format(
						"Error in UDP poll():\n%s",
						mrpt::comms::net::getLastSocketErrorStr().c_str()));
			}
			if (retval == 0)  // poll() timeout?
			{
				return INVALID_TIMESTAMP;
			}
			if ((fds[0].revents & POLLERR) || (fds[0].revents & POLLHUP) ||
				(fds[0].revents & POLLNVAL))  // device error?
			{
				THROW_EXCEPTION(
					"Error in UDP poll() (seems Velodyne device error?)");
			}
		} while ((fds[0].revents & POLLIN) == 0);

		// Receive packets that should now be available from the
		// socket using a blocking read.
		int nbytes = recvfrom(
			hSocket, (char*)&out_buffer[0], expected_packet_size, 0,
			(sockaddr*)&sender_address, &sender_address_len);

		if (nbytes < 0)
		{
			if (errno != EWOULDBLOCK) THROW_EXCEPTION("recvfrom() failed!?!");
		}
		else if ((size_t)nbytes == expected_packet_size)
		{
			// read successful,
			// if packet is not from the lidar scanner we selected by IP,
			// continue otherwise we are done
			if (!filter_only_from_IP.empty() &&
				sender_address.sin_addr.s_addr != devip_addr)
				continue;
			else
				break;  // done
		}

		std::cerr
			<< "[CVelodyneScanner] Warning: incomplete Velodyne packet read: "
			<< nbytes << " bytes\n";
	}

	// Average the times at which we begin and end reading.  Use that to
	// estimate when the scan occurred.
	const mrpt::system::TTimeStamp time2 = mrpt::system::now();

	return mrpt::Clock::time_point(mrpt::Clock::duration(
		time1.time_since_epoch().count() / 2 +
		time2.time_since_epoch().count() / 2));
}

bool CVelodyneScanner::internal_read_PCAP_packet(
	mrpt::system::TTimeStamp& data_pkt_time, uint8_t* out_data_buffer,
	mrpt::system::TTimeStamp& pos_pkt_time, uint8_t* out_pos_buffer)
{
#if MRPT_HAS_LIBPCAP
	ASSERT_(m_pcap);

	data_pkt_time = INVALID_TIMESTAMP;
	pos_pkt_time = INVALID_TIMESTAMP;

	char errbuf[PCAP_ERRBUF_SIZE];
	struct pcap_pkthdr* header;
	const u_char* pkt_data;

	while (true)
	{
		int res;
		if ((res = pcap_next_ex(
				 reinterpret_cast<pcap_t*>(m_pcap), &header, &pkt_data)) >= 0)
		{
			++m_pcap_read_count;

			// if packet is not from the lidar scanner we selected by IP,
			// continue
			if (pcap_offline_filter(
					reinterpret_cast<bpf_program*>(m_pcap_bpf_program), header,
					pkt_data) == 0)
			{
				if (m_verbose)
					std::cout
						<< "[CVelodyneScanner] DEBUG: Filtering out packet #"
						<< m_pcap_read_count << " in PCAP file.\n";
				continue;
			}

			// Determine whether it is a DATA or POSITION packet:
			m_pcap_file_empty = false;
			const mrpt::system::TTimeStamp tim = mrpt::system::now();
			const uint16_t udp_dst_port =
				ntohs(*reinterpret_cast<const uint16_t*>(pkt_data + 0x24));

			if (udp_dst_port == CVelodyneScanner::VELODYNE_POSITION_UDP_PORT)
			{
				if (m_verbose)
					std::cout << "[CVelodyneScanner] DEBUG: Packet #"
							  << m_pcap_read_count
							  << " in PCAP file is POSITION pkt.\n";
				memcpy(
					out_pos_buffer, pkt_data + 42,
					CObservationVelodyneScan::POS_PACKET_SIZE);
				pos_pkt_time = tim;  // success
				return true;
			}
			else if (udp_dst_port == CVelodyneScanner::VELODYNE_DATA_UDP_PORT)
			{
				if (m_verbose)
					std::cout << "[CVelodyneScanner] DEBUG: Packet #"
							  << m_pcap_read_count
							  << " in PCAP file is DATA pkt.\n";
				memcpy(
					out_data_buffer, pkt_data + 42,
					CObservationVelodyneScan::PACKET_SIZE);
				data_pkt_time = tim;  // success
				return true;
			}
			else
			{
				std::cerr << "[CVelodyneScanner] ERROR: Packet "
						  << m_pcap_read_count
						  << " in PCAP file passed the filter but does not "
							 "match expected UDP port numbers! Skipping it.\n";
			}
		}

		if (m_pcap_file_empty)  // no data in file?
		{
			fprintf(
				stderr,
				"[CVelodyneScanner] Maybe the PCAP file is empty? Error %d "
				"reading Velodyne packet: `%s`\n",
				res, pcap_geterr(reinterpret_cast<pcap_t*>(m_pcap)));
			return true;
		}

		if (m_pcap_read_once)
		{
			if (m_pcap_verbose)
				printf(
					"[CVelodyneScanner] INFO: end of file reached -- done "
					"reading.\n");
			std::this_thread::sleep_for(250ms);
			return false;
		}

		if (m_pcap_repeat_delay > 0.0)
		{
			if (m_pcap_verbose)
				printf(
					"[CVelodyneScanner] INFO: end of file reached -- delaying "
					"%.3f seconds.\n",
					m_pcap_repeat_delay);
			std::this_thread::sleep_for(
				std::chrono::duration<double, std::milli>(
					m_pcap_repeat_delay * 1000.0));
		}

		if (m_pcap_verbose)
			printf("[CVelodyneScanner] INFO: replaying Velodyne dump file.\n");

		// rewind the file
		pcap_close(reinterpret_cast<pcap_t*>(m_pcap));
		if ((m_pcap = pcap_open_offline(m_pcap_input_file.c_str(), errbuf)) ==
			nullptr)
		{
			THROW_EXCEPTION_FMT("Error opening PCAP file: '%s'", errbuf);
		}
		m_pcap_file_empty = true;  // maybe the file disappeared?
	}  // loop back and try again
#else
	THROW_EXCEPTION(
		"MRPT needs to be built against libpcap to enable this functionality")
#endif
}

bool CVelodyneScanner::setLidarReturnType(return_type_t ret_type)
{
	/* HTTP-based config: http://10.0.0.100/tab/config.html
		<form name='returns' method="post" action="/cgi/setting">
		<span>Return&nbsp;Type:&nbsp;</span> <select name="returns"
		onchange='javascript:this.form.submit()'>
		<option>Strongest</option>
		<option>Last</option>
		<option>Dual</option>
		</select>
		</form>
	*/
	MRPT_START;
	std::string strRet;
	switch (ret_type)
	{
		case STRONGEST:
			strRet = "Strongest";
			break;
		case DUAL:
			strRet = "Dual";
			break;
		case LAST:
			strRet = "Last";
			break;
		case UNCHANGED:
			return true;
		default:
			THROW_EXCEPTION("Invalid value for return type!");
	};

	const std::string cmd = mrpt::format("returns=%s", strRet.c_str());
	return this->internal_send_http_post(cmd);
	MRPT_END;
}

bool CVelodyneScanner::setLidarRPM(int rpm)
{
	/* HTTP-based config: http://10.0.0.100/tab/config.html
	<form name='rpm' method="post" action="/cgi/setting">
	Motor
	&nbsp;RPM:&nbsp;<input type="text" name="rpm" size="5"
	style="text-align:right" /><input type="button" value="+"
	onclick="javascript:this.form.rpm.value++;this.form.submit()" /><input
	type="button" value="-"
	onclick="javascript:this.form.rpm.value--;this.form.submit()" />
	&nbsp;<input type="submit" value="Set" />
	</form>
	*/

	MRPT_START;
	const std::string cmd = mrpt::format("rpm=%i", rpm);
	return this->internal_send_http_post(cmd);
	MRPT_END;
}

bool CVelodyneScanner::setLidarOnOff(bool on)
{
	// laser = on|off
	MRPT_START;
	const std::string cmd = mrpt::format("laser=%s", on ? "on" : "off");
	return this->internal_send_http_post(cmd);
	MRPT_END;
}

void CVelodyneScanner::setFramePublishing(bool on)
{
	// frame publishing | data packet publishing = on|off
	MRPT_START;
	m_return_frames = on;
	MRPT_END;
}

bool CVelodyneScanner::internal_send_http_post(const std::string& post_data)
{
	MRPT_START;

	ASSERTMSG_(
		!m_device_ip.empty(), "A device IP address must be specified first!");

	using namespace mrpt::comms::net;

	std::vector<uint8_t> post_out;
	string post_err_str;

	int http_rep_code;
	mrpt::system::TParameters<string> extra_headers, out_headers;

	extra_headers["Origin"] = mrpt::format("http://%s", m_device_ip.c_str());
	extra_headers["Referer"] = mrpt::format("http://%s", m_device_ip.c_str());
	extra_headers["Upgrade-Insecure-Requests"] = "1";
	extra_headers["Content-Type"] = "application/x-www-form-urlencoded";

	ERRORCODE_HTTP ret = http_request(
		"POST", post_data,
		mrpt::format("http://%s/cgi/setting", m_device_ip.c_str()), post_out,
		post_err_str, 80 /* port */, string(), string(),  // user,pass
		&http_rep_code, &extra_headers, &out_headers);

	return mrpt::comms::net::erOk == ret &&
		   (http_rep_code == 200 || http_rep_code == 204);  // OK codes

	MRPT_END;
}
