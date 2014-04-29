/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers

#include <mrpt/config.h>

#if defined(MRPT_OS_LINUX) || defined(MRPT_OS_APPLE)

#include <mrpt/utils/CClientTCPSocket.h>
#include <mrpt/utils/CMessage.h>
#include <cstring>

// Platform specific headers:
#include <sys/socket.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <netinet/in.h>

using namespace mrpt;
using namespace mrpt::utils;

/*---------------------------------------------------------------
					Constructor
 ---------------------------------------------------------------*/
CClientTCPSocket::CClientTCPSocket( )
{
	MRPT_TRY_START;
	m_hSock = -1;
	MRPT_TRY_END;
}


/*---------------------------------------------------------------
					Destructor
 ---------------------------------------------------------------*/
CClientTCPSocket::~CClientTCPSocket( )
{
	MRPT_TRY_START;

	// Close socket:
	close();

	MRPT_TRY_END;
}

/*---------------------------------------------------------------
					getLastErrorStr
 ---------------------------------------------------------------*/
std::string CClientTCPSocket::getLastErrorStr()
{
	return std::string(strerror(errno));
}


/*---------------------------------------------------------------
						close
 ---------------------------------------------------------------*/
void  CClientTCPSocket::close()
{
	// Delete socket:
	if (m_hSock != -1)
	{
		shutdown(m_hSock, SHUT_RDWR  );
		::close( m_hSock );
		m_hSock = -1;
	}
}





#endif
