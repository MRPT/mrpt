/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

#include <mrpt/base.h>  // Precompiled headers


#include <mrpt/utils/CServerTCPSocket.h>
#include <mrpt/utils/CClientTCPSocket.h>
#include <mrpt/system/os.h>
using namespace mrpt::utils;

#include <iostream>


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

	if( int(INVALID_SOCKET) == ::bind(m_serverSock,(struct sockaddr *)(&desiredIP),sizeof(desiredIP)) )
		THROW_EXCEPTION( getLastErrorStr() );

	// Put in listen mode:
	// ----------------------------
	if ( int(INVALID_SOCKET) ==  listen(m_serverSock,maxConnectionsWaiting) )
		THROW_EXCEPTION( getLastErrorStr() );

	if (m_verbose)
		printf_debug("[CServerTCPSocket::CServerTCPSocket] Listening at %s:%i\n",IPaddress.c_str(), listenPort );

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
	if (m_verbose)	printf_debug("[CServerTCPSocket::accept] Waiting incoming connections\n" );

	int selRet = ::select(
					 m_serverSock+1,// __nfds
					 &sockArr,		// Wait for read
					 NULL,			// Wait for write
					 NULL,			// Wait for except.
					 ptrTimeout);	// Timeout

	if( selRet==int(INVALID_SOCKET))
	{
		std::cerr << getLastErrorStr() << std::endl;
		return NULL;
	}

	if (selRet==0)
	{
		if (m_verbose)	printf_debug("[CServerTCPSocket::accept] Timeout waiting incoming connections\n" );

		// Timeout:
		return NULL;
	}
	else
	{
		if (m_verbose)	printf_debug("[CServerTCPSocket::accept] Incoming connection accepted\n" );

		// We have a new connection:
		CClientTCPSocket	*ret = new CClientTCPSocket();

		sockaddr_in			otherPart;
		socklen_t			otherPartSize = sizeof(otherPart);

		int aceptdSock = ::accept(
			m_serverSock,
			(struct sockaddr*)&otherPart,
			&otherPartSize );

		if (aceptdSock==int(INVALID_SOCKET))
		{
			std::cerr << getLastErrorStr() << std::endl;
			delete ret;
			return NULL;
		}

		ret->m_hSock = aceptdSock;

		ret->m_remotePartIP = std::string( inet_ntoa( otherPart.sin_addr ) );
		ret->m_remotePartPort = ntohs( otherPart.sin_port );

		printf_debug("[CServerTCPSocket::accept] Conection accepted from %s:%u\n",
			ret->m_remotePartIP.c_str(),
			ret->m_remotePartPort );

		return ret;
	}

	MRPT_END
}


