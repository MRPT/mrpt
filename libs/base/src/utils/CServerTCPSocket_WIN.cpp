/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers 


#include <MRPT/config.h>

#ifdef MRPT_OS_WINDOWS

#include <winsock.h>
#if (__BORLANDC__) || (_MSC_VER)
#	pragma comment (lib,"WS2_32.LIB")
#endif

#include <MRPT/UTILS/CServerTCPSocket.h>
#include <MRPT/UTILS/CClientTCPSocket.h>
#include <mrpt/system/os.h>

using namespace mrpt::utils;

/*---------------------------------------------------------------
					Constructor
 ---------------------------------------------------------------*/
CServerTCPSocket::CServerTCPSocket(
	unsigned short		listenPort,
	const std::string	&IPaddress,
	int					maxConnectionsWaiting,
	bool				verbose
	) :
		m_verbose(verbose)
{
	MRPT_START

	// Init the WinSock Library:
	// ----------------------------
	WORD		wVersionRequested;
	WSADATA		wsaData;

	wVersionRequested = MAKEWORD( 2, 0 );

	if ( 0 != WSAStartup( wVersionRequested, &wsaData ) )
		THROW_EXCEPTION( getLastErrorStr() );

	// Create the socket and put it listening:
	setupSocket( listenPort, IPaddress, maxConnectionsWaiting );

	MRPT_END
}


/*---------------------------------------------------------------
					getLastErrorStr
 ---------------------------------------------------------------*/
std::string CServerTCPSocket::getLastErrorStr()
{
	char buf[10];
	sprintf(buf, "%d", WSAGetLastError());
	return std::string(buf);
}

/*---------------------------------------------------------------
					Destructor
 ---------------------------------------------------------------*/
CServerTCPSocket::~CServerTCPSocket( )
{
	MRPT_START

	// Delete socket:
	if (m_serverSock != INVALID_SOCKET)
		closesocket( m_serverSock );

	WSACleanup();

	MRPT_END
}


#endif  // MRPT_OS_WINDOWS
