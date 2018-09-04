/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "comms-precomp.h"  // Precompiled headers

#include <mrpt/comms/CServerTCPSocket.h>
#include <mrpt/comms/CClientTCPSocket.h>
#include <mrpt/core/exceptions.h>

#ifdef _WIN32
// Windows
#include <winsock.h>
#if defined(_MSC_VER)
#pragma comment(lib, "WS2_32.LIB")
#endif
#else
// Linux & Apple
// Platform specific headers:
#include <sys/socket.h>
#include <unistd.h>
#include <fcntl.h>
#include <cerrno>
//#include <iostream>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#endif

using namespace mrpt;
using namespace mrpt::comms;

CServerTCPSocket::CServerTCPSocket(
	unsigned short listenPort, const std::string& IPaddress,
	int maxConnectionsWaiting, mrpt::system::VerbosityLevel verbosityLevel)
	: COutputLogger("CServerTCPSocket")
{
	MRPT_TRY_START;
	setVerbosityLevel(verbosityLevel);

#if defined(_WIN32)
	// Init the WinSock Library:
	// ----------------------------
	WORD wVersionRequested;
	WSADATA wsaData;

	wVersionRequested = MAKEWORD(2, 0);

	if (0 != WSAStartup(wVersionRequested, &wsaData))
		THROW_EXCEPTION(getLastErrorStr());
#endif

	setupSocket(listenPort, IPaddress, maxConnectionsWaiting);
	MRPT_TRY_END;
}

CServerTCPSocket::~CServerTCPSocket()
{
// Delete socket:
#if defined(_WIN32)
	if (m_serverSock != INVALID_SOCKET) closesocket(m_serverSock);
	WSACleanup();
#else
	if (m_serverSock != -1) close(m_serverSock);
#endif
}
