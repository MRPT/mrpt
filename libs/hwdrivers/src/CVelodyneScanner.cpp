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

// socket's hdrs:
#ifdef MRPT_OS_WINDOWS
	#include <winsock2.h>
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
#endif

using namespace mrpt;
using namespace mrpt::hwdrivers;
using namespace mrpt::obs;
using namespace std;

IMPLEMENTS_GENERIC_SENSOR(CVelodyneScanner,mrpt::hwdrivers)

short int CVelodyneScanner::VELODYNE_DATA_UDP_PORT = 2368;
short int CVelodyneScanner::VELODYNE_POSITION_UDP_PORT= 8308;

MRPT_TODO("Load params from cfg file")

CVelodyneScanner::CVelodyneScanner() : 
	m_model("VLP-16"),
	m_device_ip(""),
	m_rpm(600),
	m_hDataSock(INVALID_SOCKET),
	m_hPositionSock(INVALID_SOCKET)
{
	m_sensorLabel = "Velodyne_";

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
}

bool CVelodyneScanner::loadCalibrationFile(const std::string & velodyne_xml_calib_file_path )
{
	return m_velodyne_calib.loadFromXMLFile(velodyne_xml_calib_file_path);
}

void CVelodyneScanner::loadConfig_sensorSpecific(
	const mrpt::utils::CConfigFileBase &configSource,
	const std::string			&iniSection )
{
	MRPT_START

//	m_usbSerialNumber = configSource.read_string(iniSection, "USB_serialname","",false);

	MRPT_END
}

bool CVelodyneScanner::getNextObservation(
	mrpt::obs::CObservationVelodyneScanPtr & outScan,
	mrpt::obs::CObservationGPSPtr          & outGPS
	)
{
	try
	{
		// Init with empty smart pointers:
		outScan = mrpt::obs::CObservationVelodyneScanPtr();
		outGPS  = mrpt::obs::CObservationGPSPtr();
		
		// Try to get data packet:
		mrpt::obs::CObservationVelodyneScan::TVelodyneRawPacket  rx_pkt;
		mrpt::system::TTimeStamp data_pkt_timestamp = receiveDataPacket(rx_pkt);
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
				m_rx_scan->sensorLabel = this->m_sensorLabel;
				m_rx_scan->sensorPose = m_sensorPose;
				m_rx_scan->maxRange = 130.0; MRPT_TODO("Set from model");
			}
			// Accumulate pkts in the observation object:
			m_rx_scan->scan_packets.push_back(rx_pkt);
		}
		
		return true;
	}
	catch(exception &e)
	{
		cerr << "[CEnoseModular::getObservation] Returning false due to exception: " << endl;
		cerr << e.what() << endl;
		return false;
	}
	catch(...)
	{
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

void CVelodyneScanner::close()
{
	if (m_hDataSock!=INVALID_SOCKET)
	{
		shutdown(m_hDataSock, 2 ); //SD_BOTH  );
		closesocket( m_hDataSock );
		m_hDataSock=INVALID_SOCKET;
	}

	if (m_hPositionSock!=INVALID_SOCKET)
	{
		shutdown(m_hPositionSock, 2 ); //SD_BOTH  );
		closesocket( m_hPositionSock );
		m_hPositionSock=INVALID_SOCKET;
	}
}

mrpt::system::TTimeStamp CVelodyneScanner::receiveDataPacket(mrpt::obs::CObservationVelodyneScan::TVelodyneRawPacket &out_pkt)
{
	return internal_receive_UDP_packet(m_hDataSock,(uint8_t*)&out_pkt, CObservationVelodyneScan::PACKET_SIZE,m_device_ip);
}

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
	static const int POLL_TIMEOUT = 1000; // one second (in msec)

	sockaddr_in sender_address;
	int sender_address_len = sizeof(sender_address);

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

