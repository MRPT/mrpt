/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers

#include <mrpt/config.h>

#ifdef MRPT_OS_WINDOWS

#include <mrpt/utils/CClientTCPSocket.h>
#include <mrpt/utils/CMessage.h>

#include <winsock2.h>
#include <winerror.h>

#if defined(__BORLANDC__) || defined(_MSC_VER)
#pragma comment (lib,"WS2_32.LIB")
#endif


using namespace mrpt::utils;
using namespace mrpt;
using namespace std;

/*---------------------------------------------------------------
					Constructor
 ---------------------------------------------------------------*/
CClientTCPSocket::CClientTCPSocket( )
{
	MRPT_START

	// Init the WinSock Library:
	// ----------------------------
	WORD		wVersionRequested;
	WSADATA		wsaData;

	wVersionRequested = MAKEWORD( 2, 0 );

	if (WSAStartup( wVersionRequested, &wsaData ) )
		THROW_EXCEPTION("Error calling WSAStartup");

	m_hSock = INVALID_SOCKET;

	MRPT_END
}


/*---------------------------------------------------------------
					Destructor
 ---------------------------------------------------------------*/
CClientTCPSocket::~CClientTCPSocket( )
{
	// Close socket:
	close();

	WSACleanup();
}

/*---------------------------------------------------------------
						close
 ---------------------------------------------------------------*/
void  CClientTCPSocket::close()
{
	MRPT_START

	// Delete socket:
	if (m_hSock != INVALID_SOCKET)
	{
		shutdown(m_hSock, 2 ); //SD_BOTH  );
		closesocket( m_hSock );
		m_hSock = INVALID_SOCKET;
	}

	MRPT_END
}

#endif // MRPT_OS_WINDOWS
