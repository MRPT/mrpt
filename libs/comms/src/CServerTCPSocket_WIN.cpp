/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "comms-precomp.h"  // Precompiled headers

#include <MRPT/config.h>

#ifdef _WIN32

#include <winsock.h>
#if (__BORLANDC__) || (_MSC_VER)
#pragma comment(lib, "WS2_32.LIB")
#endif

#include <mrpt/comms/CServerTCPSocket.h>
#include <mrpt/comms/CClientTCPSocket.h>
#include <mrpt/system/os.h>

using namespace mrpt::utils;
using namespace mrpt::comms;

/*---------------------------------------------------------------
					Constructor
 ---------------------------------------------------------------*/
CServerTCPSocket::CServerTCPSocket(
	unsigned short listenPort, const std::string& IPaddress,
	int maxConnectionsWaiting, mrpt::utils::VerbosityLevel verbosityLevel)
	: COutputLogger("CServerTCPSocket")
{
	MRPT_START
	setVerbosityLevel(verbosityLevel);

	// Init the WinSock Library:
	// ----------------------------
	WORD wVersionRequested;
	WSADATA wsaData;

	wVersionRequested = MAKEWORD(2, 0);

	if (0 != WSAStartup(wVersionRequested, &wsaData))
		THROW_EXCEPTION(getLastErrorStr();;

	// Create the socket and put it listening:
	setupSocket(listenPort, IPaddress, maxConnectionsWaiting);

	MRPT_END
}

/*---------------------------------------------------------------
					Destructor
 ---------------------------------------------------------------*/
CServerTCPSocket::~CServerTCPSocket()
{
	// Delete socket:
	if (m_serverSock != INVALID_SOCKET) closesocket(m_serverSock);

	WSACleanup();
}

#endif  // _WIN32
