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
