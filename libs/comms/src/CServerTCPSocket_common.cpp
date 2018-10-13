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
#include <mrpt/comms/net_utils.h>
#include <mrpt/system/os.h>
#include <mrpt/core/exceptions.h>
#include <cstdio>  // stderr
using namespace mrpt::comms;

#if defined(MRPT_OS_LINUX) || defined(__APPLE__)
#define INVALID_SOCKET (-1)

#include <sys/socket.h>
#include <unistd.h>
#include <fcntl.h>
#include <cerrno>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#endif

#ifdef _WIN32
#include <winsock.h>
using socklen_t = int;
#endif

std::string CServerTCPSocket::getLastErrorStr()
{
	return mrpt::comms::net::getLastSocketErrorStr();
}

void CServerTCPSocket::setupSocket(
	unsigned short listenPort, const std::string& IPaddress,
	int maxConnectionsWaiting)
{
	MRPT_START

	// Create the socket:
	// ----------------------------
	m_serverSock = socket(AF_INET, SOCK_STREAM, 0);
	if (INVALID_SOCKET == m_serverSock) THROW_EXCEPTION(getLastErrorStr());

	// Bind it:
	// ----------------------------
	sockaddr_in desiredIP;

	desiredIP.sin_family = AF_INET;
	desiredIP.sin_addr.s_addr = inet_addr(IPaddress.c_str());
	desiredIP.sin_port = htons((unsigned short)listenPort);

	if (INVALID_SOCKET ==
		::bind(m_serverSock, (struct sockaddr*)(&desiredIP), sizeof(desiredIP)))
		THROW_EXCEPTION(getLastErrorStr());

	// Put in listen mode:
	// ----------------------------
	if (INVALID_SOCKET == listen(m_serverSock, maxConnectionsWaiting))
		THROW_EXCEPTION(getLastErrorStr());

	MRPT_LOG_DEBUG(format(
		"[CServerTCPSocket] Listening at %s:%i\n", IPaddress.c_str(),
		listenPort));

	MRPT_END
}

/*---------------------------------------------------------------
					isListening
 ---------------------------------------------------------------*/
bool CServerTCPSocket::isListening() { return INVALID_SOCKET != m_serverSock; }
/*---------------------------------------------------------------
					accept
 ---------------------------------------------------------------*/
std::unique_ptr<CClientTCPSocket> CServerTCPSocket::accept(int timeout_ms)
{
	MRPT_START

	if (m_serverSock == INVALID_SOCKET) return nullptr;

	struct timeval timeoutSelect;
	struct timeval* ptrTimeout;
	fd_set sockArr;

	// Init fd_set structure & add our socket to it:
	FD_ZERO(&sockArr);
	FD_SET(m_serverSock, &sockArr);

	// The timeout:
	if (timeout_ms < 0)
	{
		ptrTimeout = nullptr;
	}
	else
	{
		timeoutSelect.tv_sec = timeout_ms / 1000;
		timeoutSelect.tv_usec = 1000 * (timeout_ms % 1000);
		ptrTimeout = &timeoutSelect;
	}

	// Wait for READ flag (meaning incoming connections):
	MRPT_LOG_DEBUG("[CServerTCPSocket::accept] Waiting incoming connections");

	int selRet = ::select(
		m_serverSock + 1,  // __nfds
		&sockArr,  // Wait for read
		nullptr,  // Wait for write
		nullptr,  // Wait for except.
		ptrTimeout);  // Timeout

	if (selRet == INVALID_SOCKET)
	{
		fprintf(stderr, "%s\n", getLastErrorStr().c_str());
		return nullptr;
	}

	if (selRet == 0)
	{
		MRPT_LOG_WARN(
			"[CServerTCPSocket::accept] Timeout waiting incoming "
			"connections\n");

		// Timeout:
		return nullptr;
	}
	else
	{
		MRPT_LOG_DEBUG(
			"[CServerTCPSocket::accept] Incoming connection accepted\n");

		// We have a new connection:

		sockaddr_in otherPart;
		socklen_t otherPartSize = sizeof(otherPart);

		int aceptdSock = ::accept(
			m_serverSock, (struct sockaddr*)&otherPart, &otherPartSize);

		if (aceptdSock == INVALID_SOCKET)
		{
			MRPT_LOG_ERROR_FMT("%s\n", getLastErrorStr().c_str());
			return std::unique_ptr<CClientTCPSocket>();
		}

		auto ret = std::make_unique<CClientTCPSocket>();

		ret->m_hSock = aceptdSock;

		ret->m_remotePartIP = std::string(inet_ntoa(otherPart.sin_addr));
		ret->m_remotePartPort = ntohs(otherPart.sin_port);

		MRPT_LOG_DEBUG_FMT(
			"[CServerTCPSocket::accept] Connection accepted from %s:%u\n",
			ret->m_remotePartIP.c_str(), ret->m_remotePartPort);

		return ret;
	}

	MRPT_END
}
