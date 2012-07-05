/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
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

		if ( m_verbose )
			printf_debug("[CServerTCPSocket::accept] Conection accepted from %s:%u\n",
				ret->m_remotePartIP.c_str(),
				ret->m_remotePartPort );

		return ret;
	}

	MRPT_END
}


