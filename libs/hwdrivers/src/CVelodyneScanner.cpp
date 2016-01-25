/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2015, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "hwdrivers-precomp.h"   // Precompiled headers

#include <mrpt/hwdrivers/CVelodyneScanner.h>
#include <mrpt/utils/CStream.h>
#include <mrpt/utils/net_utils.h>
#include <mrpt/hwdrivers/CGPSInterface.h>

MRPT_TODO("Add unit tests")
MRPT_TODO("Optional save to PCAP")
MRPT_TODO("rpm: document usage. Add automatic determination of rpm from number of pkts/scan")

// socket's hdrs:
#ifdef MRPT_OS_WINDOWS
	#include <winsock2.h>
	typedef int socklen_t;
#else
	#define  INVALID_SOCKET  (-1)
	#include <sys/socket.h>
	#include <unistd.h>
	#include <fcntl.h>
	#include <errno.h>
	#include <sys/types.h>
	#include <sys/ioctl.h>
	#include <netdb.h>
	#include <arpa/inet.h>
	#include <netinet/in.h>
	#include <netinet/tcp.h>
	#include <poll.h>
#endif

#if MRPT_HAS_LIBPCAP
#	include <pcap.h>
#	if !defined(PCAP_NETMASK_UNKNOWN)  // for older pcap versions
#	define PCAP_NETMASK_UNKNOWN    0xffffffff
#	endif
#endif

using namespace mrpt;
using namespace mrpt::hwdrivers;
using namespace mrpt::obs;
using namespace std;

IMPLEMENTS_GENERIC_SENSOR(CVelodyneScanner,mrpt::hwdrivers)

short int CVelodyneScanner::VELODYNE_DATA_UDP_PORT = 2368;
short int CVelodyneScanner::VELODYNE_POSITION_UDP_PORT= 8308;

// Hard-wired properties of LIDARs depending on the model:
struct TModelProperties {
	double maxRange;
};
struct TModelPropertiesFactory {
	static const std::map<std::string,TModelProperties>  & get()
	{
		static std::map<std::string,TModelProperties>  modelProperties;
		static bool init = false;
		if (!init) {
			init = true;
			modelProperties["VLP-16"].maxRange = 120.0;
			modelProperties["HDL-32"].maxRange = 70.0;
		}
		return modelProperties;
	}
	// Return human-readable string: "`VLP-16`,`XXX`,..."
	static std::string getListKnownModels(){
		const std::map<std::string,TModelProperties>  & lst = TModelPropertiesFactory::get();
		std::string s;
		for (std::map<std::string,TModelProperties>::const_iterator it=lst.begin();it!=lst.end();++it)
			s+=mrpt::format("`%s`,",it->first.c_str());
		return s;
	}
};


CVelodyneScanner::CVelodyneScanner() :
	m_model("VLP-16"),
	m_device_ip(""),
	m_rpm(600),
	m_pcap(NULL),
	m_pcap_bpf_program(NULL),
	m_pcap_file_empty(true),
	m_pcap_read_once(false),
	m_pcap_read_fast(false),
	m_pcap_repeat_delay(0.0),
	m_hDataSock(INVALID_SOCKET),
	m_hPositionSock(INVALID_SOCKET)
{
	m_sensorLabel = "Velodyne";

#if MRPT_HAS_LIBPCAP
	m_pcap_bpf_program = new bpf_program[1];
#endif

#ifdef MRPT_OS_WINDOWS
	// Init the WinSock Library:
	WORD    wVersionRequested = MAKEWORD( 2, 0 );
	WSADATA wsaData;
	if (WSAStartup( wVersionRequested, &wsaData ) )
		THROW_EXCEPTION("Error calling WSAStartup");
#endif
}

CVelodyneScanner::~CVelodyneScanner( )
{
	this->close();
#ifdef MRPT_OS_WINDOWS
	WSACleanup();
#endif

#if MRPT_HAS_LIBPCAP
	delete[] reinterpret_cast<bpf_program*>(m_pcap_bpf_program);
	m_pcap_bpf_program=NULL;
#endif

}

bool CVelodyneScanner::loadCalibrationFile(const std::string & velodyne_xml_calib_file_path )
{
	return m_velodyne_calib.loadFromXMLFile(velodyne_xml_calib_file_path);
}

void CVelodyneScanner::loadConfig_sensorSpecific(
	const mrpt::utils::CConfigFileBase &cfg,
	const std::string			&sect )
{
	MRPT_START

	MRPT_LOAD_HERE_CONFIG_VAR(device_ip,   string, m_device_ip       ,  cfg, sect);
	MRPT_LOAD_HERE_CONFIG_VAR(rpm,            int, m_rpm             ,  cfg, sect);
	MRPT_LOAD_HERE_CONFIG_VAR(model,       string, m_model           ,  cfg, sect);
	MRPT_LOAD_HERE_CONFIG_VAR(pcap_input,  string, m_pcap_input_file ,  cfg, sect);
	MRPT_LOAD_HERE_CONFIG_VAR(pcap_read_once,bool, m_pcap_read_once,  cfg, sect);
	MRPT_LOAD_HERE_CONFIG_VAR(pcap_read_fast,bool, m_pcap_read_fast ,  cfg, sect);
	MRPT_LOAD_HERE_CONFIG_VAR(pcap_repeat_delay,double, m_pcap_repeat_delay ,  cfg, sect);

	std::string calibration_file;
	MRPT_LOAD_CONFIG_VAR(calibration_file,string,cfg, sect);
	if (!calibration_file.empty())
		this->loadCalibrationFile(calibration_file);

	// Check validity:
	const std::map<std::string,TModelProperties> &lstModels = TModelPropertiesFactory::get();
	if (lstModels.find(m_model)==lstModels.end()) {
		THROW_EXCEPTION(mrpt::format("Unrecognized `model` parameter: `%s` . Known values are: %s",m_model.c_str(),TModelPropertiesFactory::getListKnownModels().c_str()))
	}

	MRPT_END
}

bool CVelodyneScanner::getNextObservation(
	mrpt::obs::CObservationVelodyneScanPtr & outScan,
	mrpt::obs::CObservationGPSPtr          & outGPS
	)
{
	try
	{
		ASSERTMSG_(m_initialized, "initialize() has not been called yet!");

		// Init with empty smart pointers:
		outScan = mrpt::obs::CObservationVelodyneScanPtr();
		outGPS  = mrpt::obs::CObservationGPSPtr();

		// Try to get data & pos packets:
		mrpt::obs::CObservationVelodyneScan::TVelodyneRawPacket  rx_pkt;
		mrpt::obs::CObservationVelodyneScan::TVelodynePositionPacket rx_pos_pkt;
		mrpt::system::TTimeStamp data_pkt_timestamp, pos_pkt_timestamp;
		this->receivePackets(data_pkt_timestamp,rx_pkt, pos_pkt_timestamp,rx_pos_pkt);

		if (pos_pkt_timestamp!=INVALID_TIMESTAMP)
		{
			mrpt::obs::CObservationGPSPtr gps_obs = mrpt::obs::CObservationGPS::Create();
			gps_obs->timestamp = pos_pkt_timestamp;
			gps_obs->originalReceivedTimestamp = pos_pkt_timestamp;
			gps_obs->sensorLabel = this->m_sensorLabel + std::string("_GPS");
			gps_obs->sensorPose = m_sensorPose;

			CGPSInterface::parse_NMEA( std::string(rx_pos_pkt.NMEA_GPRMC), *gps_obs);
			outGPS = gps_obs;
		}

		if (data_pkt_timestamp!=INVALID_TIMESTAMP)
		{
			// Break into a new observation object when the azimuth passes 360->0 deg:
			if (m_rx_scan &&
			    !m_rx_scan->scan_packets.empty() &&
			    rx_pkt.blocks[0].rotation < m_rx_scan->scan_packets.rbegin()->blocks[0].rotation )
			{
				// Return the observation as done when a complete 360 deg scan is ready:
				outScan = m_rx_scan;
				m_rx_scan.clear_unique();
			}

			// Create smart ptr to new in-progress observation:
			if (!m_rx_scan) {
				m_rx_scan= mrpt::obs::CObservationVelodyneScan::Create();
				m_rx_scan->timestamp = data_pkt_timestamp;
				m_rx_scan->sensorLabel = this->m_sensorLabel + std::string("_SCAN");
				m_rx_scan->sensorPose = m_sensorPose;
				m_rx_scan->calibration = m_velodyne_calib; // Embed a copy of the calibration info

				{
					const std::map<std::string,TModelProperties> &lstModels = TModelPropertiesFactory::get();
					std::map<std::string,TModelProperties>::const_iterator it=lstModels.find(this->m_model);
					if (it!=lstModels.end())
					{ // Model params:
						m_rx_scan->maxRange = it->second.maxRange;
					}
					else // default params:
					{
						m_rx_scan->maxRange = 120.0;
					}
				}
			}
			// Accumulate pkts in the observation object:
			m_rx_scan->scan_packets.push_back(rx_pkt);
		}

		return true;
	}
	catch(exception &e)
	{
		cerr << "[CVelodyneScanner::getObservation] Returning false due to exception: " << endl;
		cerr << e.what() << endl;
		return false;
	}
}

void CVelodyneScanner::doProcess()
{
	CObservationVelodyneScanPtr obs;
	CObservationGPSPtr          obs_gps;

	if (getNextObservation(obs,obs_gps))
	{
		m_state = ssWorking;
		if (obs)     appendObservation( obs );
		if (obs_gps) appendObservation( obs_gps );
	}
	else
	{
		m_state = ssError;
		cerr << "ERROR receiving data from Velodyne devic!" << endl;
	}
}

/** Tries to initialize the sensor, after setting all the parameters with a call to loadConfig.
*  \exception This method must throw an exception with a descriptive message if some critical error is found. */
void CVelodyneScanner::initialize()
{
	this->close();

	// (0) Preparation:
	// --------------------------------
	// Make sure we have calibration data:
	if (m_velodyne_calib.empty() && m_model.empty())
		THROW_EXCEPTION("You must provide either a `model` name or load a valid XML configuration file first.");
	if (m_velodyne_calib.empty()) {
		// Try to load default data:
		m_velodyne_calib = VelodyneCalibration::LoadDefaultCalibration(m_model);
		if (m_velodyne_calib.empty())
			THROW_EXCEPTION("Could not find default calibration data for the given LIDAR `model` name. Please, specify a valid `model` or load a valid XML configuration file first.");
	}

	// online vs off line operation??
	// -------------------------------
	if (m_pcap_input_file.empty())
	{ // Online

		// (1) Create LIDAR DATA socket
		// --------------------------------
		if ( INVALID_SOCKET == (m_hDataSock = socket(PF_INET, SOCK_DGRAM, 0)) )
			THROW_EXCEPTION( format("Error creating UDP socket:\n%s", mrpt::utils::net::getLastSocketErrorStr().c_str() ));

		struct sockaddr_in  bindAddr;
		memset(&bindAddr,0,sizeof(bindAddr));
		bindAddr.sin_family  = AF_INET;
		bindAddr.sin_port    = htons(VELODYNE_DATA_UDP_PORT);
		bindAddr.sin_addr.s_addr = INADDR_ANY;

		if( int(INVALID_SOCKET) == ::bind(m_hDataSock,(struct sockaddr *)(&bindAddr),sizeof(sockaddr)) )
			THROW_EXCEPTION( mrpt::utils::net::getLastSocketErrorStr() );

#ifdef MRPT_OS_WINDOWS
		unsigned long non_block_mode = 1;
		if (ioctlsocket(m_hDataSock, FIONBIO, &non_block_mode) ) THROW_EXCEPTION( "Error entering non-blocking mode with ioctlsocket()." );
#else
		int oldflags=fcntl(m_hDataSock, F_GETFL, 0);
		if (oldflags == -1)  THROW_EXCEPTION( "Error retrieving fcntl() of socket." );
		oldflags |= O_NONBLOCK | FASYNC;
		if (-1==fcntl(m_hDataSock, F_SETFL, oldflags))  THROW_EXCEPTION( "Error entering non-blocking mode with fcntl()." );
#endif

		// (2) Create LIDAR POSITION socket
		// --------------------------------
		if ( INVALID_SOCKET == (m_hPositionSock = socket(PF_INET, SOCK_DGRAM, 0)) )
			THROW_EXCEPTION( format("Error creating UDP socket:\n%s", mrpt::utils::net::getLastSocketErrorStr().c_str() ));

		bindAddr.sin_port    = htons(VELODYNE_POSITION_UDP_PORT);

		if( int(INVALID_SOCKET) == ::bind(m_hPositionSock,(struct sockaddr *)(&bindAddr),sizeof(sockaddr)) )
			THROW_EXCEPTION( mrpt::utils::net::getLastSocketErrorStr() );

#ifdef MRPT_OS_WINDOWS
		if (ioctlsocket(m_hPositionSock, FIONBIO, &non_block_mode) ) THROW_EXCEPTION( "Error entering non-blocking mode with ioctlsocket()." );
#else
		oldflags=fcntl(m_hPositionSock, F_GETFL, 0);
		if (oldflags == -1)  THROW_EXCEPTION( "Error retrieving fcntl() of socket." );
		oldflags |= O_NONBLOCK | FASYNC;
		if (-1==fcntl(m_hPositionSock, F_SETFL, oldflags))  THROW_EXCEPTION( "Error entering non-blocking mode with fcntl()." );
#endif
	}
	else
	{ // Offline:
#if MRPT_HAS_LIBPCAP
		char errbuf[PCAP_ERRBUF_SIZE];

		printf("\n[CVelodyneScanner] Opening PCAP file \"%s\"\n", m_pcap_input_file.c_str());
		if ((m_pcap = pcap_open_offline(m_pcap_input_file.c_str(), errbuf) ) == NULL) {
			THROW_EXCEPTION_CUSTOM_MSG1("Error opening PCAP file: '%s'",errbuf);
		}

		// Build PCAP filter:
		{
			std::string filter_str =
				mrpt::format("(udp dst port %d || udp dst port %d)",
					static_cast<int>(CVelodyneScanner::VELODYNE_DATA_UDP_PORT),
					static_cast<int>(CVelodyneScanner::VELODYNE_POSITION_UDP_PORT) );
			if( !m_device_ip.empty() )
				filter_str += "&& src host " + m_device_ip;

			if (pcap_compile( reinterpret_cast<pcap_t*>(m_pcap), reinterpret_cast<bpf_program*>(m_pcap_bpf_program), filter_str.c_str(), 1, PCAP_NETMASK_UNKNOWN) <0)
				pcap_perror(reinterpret_cast<pcap_t*>(m_pcap),"[CVelodyneScanner] Error calling pcap_compile: ");
		}

		m_pcap_file_empty = true;
		m_pcap_read_count = 0;

#else
		THROW_EXCEPTION("Velodyne: Reading from PCAP requires building MRPT with libpcap support!");
#endif
	}

	m_initialized=true;
}

void CVelodyneScanner::close()
{
	if (m_hDataSock!=INVALID_SOCKET)
	{
		shutdown(m_hDataSock, 2 ); //SD_BOTH  );
#ifdef MRPT_OS_WINDOWS
		closesocket( m_hDataSock );
#else
		::close( m_hDataSock );
#endif
		m_hDataSock=INVALID_SOCKET;
	}

	if (m_hPositionSock!=INVALID_SOCKET)
	{
		shutdown(m_hPositionSock, 2 ); //SD_BOTH  );
#ifdef MRPT_OS_WINDOWS
		closesocket( m_hDataSock );
#else
		::close( m_hDataSock );
#endif
		m_hPositionSock=INVALID_SOCKET;
	}
#if MRPT_HAS_LIBPCAP
	if (m_pcap)
	{
		pcap_close( reinterpret_cast<pcap_t*>(m_pcap) );
		m_pcap = NULL;
	}
#endif
	m_initialized=false;
}

void CVelodyneScanner::receivePackets(
	mrpt::system::TTimeStamp  &data_pkt_timestamp,
	mrpt::obs::CObservationVelodyneScan::TVelodyneRawPacket &out_data_pkt,
	mrpt::system::TTimeStamp  &pos_pkt_timestamp,
	mrpt::obs::CObservationVelodyneScan::TVelodynePositionPacket &out_pos_pkt
	)
{
	MRPT_COMPILE_TIME_ASSERT(sizeof(mrpt::obs::CObservationVelodyneScan::TVelodynePositionPacket)== CObservationVelodyneScan::POS_PACKET_SIZE);
	MRPT_COMPILE_TIME_ASSERT(sizeof(mrpt::obs::CObservationVelodyneScan::TVelodyneRawPacket)== CObservationVelodyneScan::PACKET_SIZE);

	if (m_pcap)
	{
		internal_read_PCAP_packet(
			data_pkt_timestamp, (uint8_t*)&out_data_pkt,
			pos_pkt_timestamp, (uint8_t*)&out_pos_pkt);
	}
	else
	{
		data_pkt_timestamp = internal_receive_UDP_packet(m_hDataSock     ,(uint8_t*)&out_data_pkt, CObservationVelodyneScan::PACKET_SIZE,m_device_ip);
		pos_pkt_timestamp  = internal_receive_UDP_packet(m_hPositionSock ,(uint8_t*)&out_pos_pkt, CObservationVelodyneScan::POS_PACKET_SIZE,m_device_ip);
	}
}

// static method:
mrpt::system::TTimeStamp CVelodyneScanner::internal_receive_UDP_packet(
	platform_socket_t   hSocket,
	uint8_t           * out_buffer,
	const size_t expected_packet_size,
	const std::string &filter_only_from_IP)
{
	if (hSocket==INVALID_SOCKET)
		THROW_EXCEPTION("Error: UDP socket is not open yet! Have you called initialize() first?");

	unsigned long devip_addr = 0;
	if (!filter_only_from_IP.empty())
		devip_addr = inet_addr(filter_only_from_IP.c_str());

	const mrpt::system::TTimeStamp time1 = mrpt::system::now();

	struct pollfd fds[1];
	fds[0].fd = hSocket;
	fds[0].events = POLLIN;
	static const int POLL_TIMEOUT = 1; // (ms)

	sockaddr_in sender_address;
	socklen_t sender_address_len = sizeof(sender_address);

	while (true)
	{
		// Unfortunately, the Linux kernel recvfrom() implementation
		// uses a non-interruptible sleep() when waiting for data,
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
#if !defined(MRPT_OS_WINDOWS)
				poll
#else
				WSAPoll
#endif
				(fds, 1, POLL_TIMEOUT);
			if (retval < 0)             // poll() error?
			{
				if (errno != EINTR)
					THROW_EXCEPTION( format("Error in UDP poll():\n%s", mrpt::utils::net::getLastSocketErrorStr().c_str() ));
			}
			if (retval == 0)            // poll() timeout?
			{
				return INVALID_TIMESTAMP;
			}
			if ((fds[0].revents & POLLERR)
				|| (fds[0].revents & POLLHUP)
				|| (fds[0].revents & POLLNVAL)) // device error?
			{
				THROW_EXCEPTION( "Error in UDP poll() (seems Velodyne device error?)");
			}
		} while ((fds[0].revents & POLLIN) == 0);

		// Receive packets that should now be available from the
		// socket using a blocking read.
		int nbytes = recvfrom(hSocket, (char*)&out_buffer[0],
			expected_packet_size,  0,
			(sockaddr*) &sender_address, &sender_address_len);

		if (nbytes < 0)
		{
			if (errno != EWOULDBLOCK)
				THROW_EXCEPTION("recvfrom() failed!?!")
		}
		else if ((size_t) nbytes == expected_packet_size)
		{
			// read successful,
			// if packet is not from the lidar scanner we selected by IP,
			// continue otherwise we are done
			if( !filter_only_from_IP.empty() && sender_address.sin_addr.s_addr != devip_addr )
				continue;
			else
				break; //done
		}

		std::cerr << "[CVelodyneScanner] Warning: incomplete Velodyne packet read: " << nbytes << " bytes\n";
	}

	// Average the times at which we begin and end reading.  Use that to
	// estimate when the scan occurred.
	const mrpt::system::TTimeStamp time2 = mrpt::system::now();

	return (time1/2+time2/2);
}

void CVelodyneScanner::internal_read_PCAP_packet(
	mrpt::system::TTimeStamp  & data_pkt_time, uint8_t *out_data_buffer,
	mrpt::system::TTimeStamp  & pos_pkt_time, uint8_t  *out_pos_buffer
	)
{
#if MRPT_HAS_LIBPCAP
	ASSERT_(m_pcap);

	data_pkt_time = INVALID_TIMESTAMP;
	pos_pkt_time  = INVALID_TIMESTAMP;

	char errbuf[PCAP_ERRBUF_SIZE];
	struct pcap_pkthdr *header;
	const u_char *pkt_data;

	while (true)
	{
		int res;
		if ((res = pcap_next_ex(reinterpret_cast<pcap_t*>(m_pcap), &header, &pkt_data)) >= 0)
		{
			++m_pcap_read_count;

			// if packet is not from the lidar scanner we selected by IP, continue
			if(pcap_offline_filter( reinterpret_cast<bpf_program*>(m_pcap_bpf_program), header, pkt_data ) == 0)
			{
				if (m_verbose) std::cout << "[CVelodyneScanner] DEBUG: Filtering out packet #"<< m_pcap_read_count <<" in PCAP file.\n";
				continue;
			}

			// Keep the reader from blowing through the file.
			if (!m_pcap_read_fast)
				mrpt::system::sleep(1); //packet_rate_.sleep();

			// Determine whether it is a DATA or POSITION packet:
			m_pcap_file_empty = false;
			const mrpt::system::TTimeStamp tim = mrpt::system::now();
			const uint16_t udp_dst_port = ntohs( *reinterpret_cast<const uint16_t *>(pkt_data + 0x24) );

			if (udp_dst_port==CVelodyneScanner::VELODYNE_POSITION_UDP_PORT) {
				if (m_verbose) std::cout << "[CVelodyneScanner] DEBUG: Packet #"<< m_pcap_read_count <<" in PCAP file is POSITION pkt.\n";
				memcpy(out_pos_buffer, pkt_data+42, CObservationVelodyneScan::POS_PACKET_SIZE);
				pos_pkt_time = tim;  // success
				return;
			}
			else if (udp_dst_port==CVelodyneScanner::VELODYNE_DATA_UDP_PORT) {
				if (m_verbose) std::cout << "[CVelodyneScanner] DEBUG: Packet #"<< m_pcap_read_count <<" in PCAP file is DATA pkt.\n";
				memcpy(out_data_buffer, pkt_data+42, CObservationVelodyneScan::PACKET_SIZE);
				data_pkt_time = tim;  // success
				return;
			}
			else {
				std::cerr << "[CVelodyneScanner] ERROR: Packet "<<m_pcap_read_count <<" in PCAP file passed the filter but does not match expected UDP port numbers! Skipping it.\n";
			}
		}

		if (m_pcap_file_empty) // no data in file?
		{
			fprintf(stderr, "[CVelodyneScanner] Maybe the PCAP file is empty? Error %d reading Velodyne packet: `%s`\n", res, pcap_geterr(reinterpret_cast<pcap_t*>(m_pcap) ));
			return;
		}

		if (m_pcap_read_once)
		{
			printf("[CVelodyneScanner] INFO: end of file reached -- done reading.");
			return;
		}

		if (m_pcap_repeat_delay > 0.0)
		{
			printf("[CVelodyneScanner] INFO: end of file reached -- delaying %.3f seconds.", m_pcap_repeat_delay);
			mrpt::system::sleep( m_pcap_repeat_delay * 1000.0);
		}

		printf("[CVelodyneScanner] INFO: replaying Velodyne dump file\n");

		// rewind the file
		pcap_close( reinterpret_cast<pcap_t*>(m_pcap) );
		if ((m_pcap = pcap_open_offline(m_pcap_input_file.c_str(), errbuf) ) == NULL) {
			THROW_EXCEPTION_CUSTOM_MSG1("Error opening PCAP file: '%s'",errbuf);
		}
		m_pcap_file_empty = true;              // maybe the file disappeared?
	} // loop back and try again
#else
	THROW_EXCEPTION("MRPT needs to be built against libpcap to enable this functionality")
#endif
}


