/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers


#include <mrpt/utils/CServerTCPSocket.h>
#include <mrpt/utils/CClientTCPSocket.h>
#include <mrpt/system/os.h>
#include <cstdio> // stderr
using namespace mrpt::utils;

#if defined(MRPT_OS_LINUX) || defined(MRPT_OS_APPLE)
	#define  INVALID_SOCKET		(-1)

	#include <sys/socket.h>
	#include <unistd.h>
	#include <fcntl.h>
	#include <errno.h>
	#include <sys/types.h>
	#include <sys/ioctl.h>
	#include <netdb.h>
	#include <arpa/inet.h>
	#include <netinet/in.h>
#endif

#ifdef MRPT_OS_WINDOWS
	#include <winsock.h>
	typedef int socklen_t;
#endif


/*---------------------------------------------------------------
					setupSocket
 ---------------------------------------------------------------*/
void CServerTCPSocket::setupSocket(
	unsigned short	  listenPort,
	const std::string &IPaddress,
	int               maxConnectionsWaiting )
{
	MRPT_START

	// Create the socket:
	// ----------------------------
	m_serverSock = socket(AF_INET, SOCK_STREAM, 0);
	if( INVALID_SOCKET == m_serverSock )
		THROW_EXCEPTION( getLastErrorStr() );

	// Bind it:
	// ----------------------------
	sockaddr_in		desiredIP;

	desiredIP.sin_family		= AF_INET;
	desiredIP.sin_addr.s_addr	= inet_addr( IPaddress.c_str() );
	desiredIP.sin_port			= htons((unsigned short)listenPort);

	if( INVALID_SOCKET == ::bind(m_serverSock,(struct sockaddr *)(&desiredIP),sizeof(desiredIP)) )
		THROW_EXCEPTION( getLastErrorStr() );

	// Put in listen mode:
	// ----------------------------
	if ( INVALID_SOCKET ==  listen(m_serverSock,maxConnectionsWaiting) )
		THROW_EXCEPTION( getLastErrorStr() );

	MRPT_LOG_DEBUG( format("[CServerTCPSocket] Listening at %s:%i\n",IPaddress.c_str(), listenPort ));

	MRPT_END
}



/*---------------------------------------------------------------
					isListening
 ---------------------------------------------------------------*/
bool CServerTCPSocket::isListening()
{
	return INVALID_SOCKET != m_serverSock;
}

/*---------------------------------------------------------------
					accept
 ---------------------------------------------------------------*/
CClientTCPSocket *  CServerTCPSocket::accept( int timeout_ms )
{
	MRPT_START

	if( m_serverSock == INVALID_SOCKET) return NULL;

	struct timeval	timeoutSelect;
	struct timeval	*ptrTimeout;
	fd_set			sockArr;

    // Init fd_set structure & add our socket to it:
    FD_ZERO(&sockArr);
    FD_SET(m_serverSock, &sockArr);

	// The timeout:
	if (timeout_ms<0)
	{
		ptrTimeout = NULL;
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
					 m_serverSock+1,// __nfds
					 &sockArr,		// Wait for read
					 NULL,			// Wait for write
					 NULL,			// Wait for except.
					 ptrTimeout);	// Timeout

	if( selRet==INVALID_SOCKET)
	{
	    fprintf(stderr,"%s\n", getLastErrorStr().c_str());
		return NULL;
	}

	if (selRet==0)
	{
		MRPT_LOG_WARN("[CServerTCPSocket::accept] Timeout waiting incoming connections\n" );

		// Timeout:
		return NULL;
	}
	else
	{
		MRPT_LOG_DEBUG("[CServerTCPSocket::accept] Incoming connection accepted\n" );

		// We have a new connection:
		CClientTCPSocket	*ret = new CClientTCPSocket();

		sockaddr_in			otherPart;
		socklen_t			otherPartSize = sizeof(otherPart);

		int aceptdSock = ::accept(
			m_serverSock,
			(struct sockaddr*)&otherPart,
			&otherPartSize );

		if (aceptdSock==INVALID_SOCKET)
		{
			fprintf(stderr,"%s\n",getLastErrorStr().c_str());
			delete ret;
			return NULL;
		}

		ret->m_hSock = aceptdSock;

		ret->m_remotePartIP = std::string( inet_ntoa( otherPart.sin_addr ) );
		ret->m_remotePartPort = ntohs( otherPart.sin_port );

		MRPT_LOG_DEBUG(format("[CServerTCPSocket::accept] Conection accepted from %s:%u\n",
				ret->m_remotePartIP.c_str(),
				ret->m_remotePartPort ) );

		return ret;
	}

	MRPT_END
}


