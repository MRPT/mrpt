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

#include <mrpt/base.h>  // Only for precomp. headers, include all libmrpt-core headers.


#include <mrpt/config.h>

#if defined(MRPT_OS_LINUX) || defined(MRPT_OS_APPLE)

#include <mrpt/utils/CServerTCPSocket.h>
#include <mrpt/utils/CClientTCPSocket.h>
#include <mrpt/system.h>

// Platform specific headers:
#include <sys/socket.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <iostream>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>


using namespace mrpt;
using namespace mrpt::utils;

/*---------------------------------------------------------------
					Constructor
 ---------------------------------------------------------------*/
CServerTCPSocket::CServerTCPSocket(
	unsigned short		listenPort,
	const std::string	&IPaddress,
	int               	maxConnectionsWaiting,
	bool				verbose
	) :
		m_verbose(verbose)
{
	MRPT_TRY_START;
	setupSocket( listenPort, IPaddress, maxConnectionsWaiting );
	MRPT_TRY_END;
}


/*---------------------------------------------------------------
					getLastErrorStr
 ---------------------------------------------------------------*/
std::string CServerTCPSocket::getLastErrorStr()
{
	return std::string(strerror(errno));
}

/*---------------------------------------------------------------
					Destructor
 ---------------------------------------------------------------*/
CServerTCPSocket::~CServerTCPSocket( )
{
	MRPT_TRY_START;

	// Delete socket:
	if (m_serverSock != -1)
		close( m_serverSock );

	MRPT_TRY_END;
}



#endif // Linux
