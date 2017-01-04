/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
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

#include <mrpt/utils/CServerTCPSocket.h>
#include <mrpt/utils/CClientTCPSocket.h>
#include <mrpt/system/os.h>

using namespace mrpt::utils;

/*---------------------------------------------------------------
					Constructor
 ---------------------------------------------------------------*/
CServerTCPSocket::CServerTCPSocket(
	unsigned short		listenPort,
	const std::string	&IPaddress,
	int					maxConnectionsWaiting,
	mrpt::utils::VerbosityLevel verbosityLevel
	) :
		COutputLogger("CServerTCPSocket")
{
	MRPT_START
	setVerbosityLevel(verbosityLevel);

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
					Destructor
 ---------------------------------------------------------------*/
CServerTCPSocket::~CServerTCPSocket( )
{
	// Delete socket:
	if (m_serverSock != INVALID_SOCKET)
		closesocket( m_serverSock );

	WSACleanup();
}


#endif  // MRPT_OS_WINDOWS
