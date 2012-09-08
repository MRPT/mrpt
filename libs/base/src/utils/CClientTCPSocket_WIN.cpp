/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | Copyright (c) 2012, University of Almeria                                 |
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


#include <MRPT/config.h>

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
	MRPT_START

	// Close socket:
	close();

	WSACleanup();

	MRPT_END
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

/*---------------------------------------------------------------
					getLastErrorStr
 ---------------------------------------------------------------*/
std::string CClientTCPSocket::getLastErrorStr()
{
	const int errnum = WSAGetLastError();

	string s;

	switch(errnum)
	{
	case WSA_INVALID_HANDLE: s="Specified event object handle is invalid."; break;
	case WSA_NOT_ENOUGH_MEMORY: s="Insufficient memory available."; break;
	case WSA_INVALID_PARAMETER: s="One or more parameters are invalid."; break;
	case WSA_OPERATION_ABORTED: s="Overlapped operation aborted."; break;
	case WSA_IO_INCOMPLETE: s="Overlapped I/O event object not in signaled state."; break;
	case WSA_IO_PENDING: s="Overlapped operations will complete later."; break;
	case WSAEINTR: s="Interrupted function call."; break;
	case WSAEBADF: s="File handle is not valid."; break;
	case WSAEACCES: s="Permission denied."; break;
	case WSAEFAULT: s="Bad address."; break;
	case WSAEINVAL: s="Invalid argument."; break;
	case WSAEMFILE: s="Too many open files."; break;
	case WSAEWOULDBLOCK: s="Resource temporarily unavailable."; break;
	case WSAEINPROGRESS: s="Operation now in progress."; break;
	case WSAEALREADY: s="Operation already in progress."; break;
	case WSAENOTSOCK: s="Socket operation on nonsocket."; break;
	case WSAEDESTADDRREQ: s="Destination address required."; break;
	case WSAEMSGSIZE: s="Message too long."; break;
	case WSAEPROTOTYPE: s="Protocol wrong type for socket."; break;
	case WSAENOPROTOOPT: s="Bad protocol option."; break;
	case WSAEPROTONOSUPPORT: s="Protocol not supported."; break;
	case WSAESOCKTNOSUPPORT: s="Socket type not supported."; break;
	case WSAEOPNOTSUPP: s="Operation not supported."; break;
	case WSAEPFNOSUPPORT: s="Protocol family not supported."; break;
	case WSAEAFNOSUPPORT: s="Address family not supported by protocol family."; break;
	case WSAEADDRINUSE: s="Address already in use."; break;
	case WSAEADDRNOTAVAIL: s="Cannot assign requested address."; break;
	case WSAENETDOWN: s="Network is down."; break;
	case WSAENETUNREACH: s="Network is unreachable."; break;
	case WSAENETRESET: s="Network dropped connection on reset."; break;
	case WSAECONNABORTED: s="Software caused connection abort."; break;
	case WSAECONNRESET: s="Connection reset by peer."; break;
	case WSAENOBUFS: s="No buffer space available."; break;
	case WSAEISCONN: s="Socket is already connected."; break;
	case WSAENOTCONN: s="Socket is not connected."; break;
	case WSAESHUTDOWN: s="Cannot send after socket shutdown."; break;
	case WSAETOOMANYREFS: s="Too many references."; break;
	case WSAETIMEDOUT: s="Connection timed out."; break;
	case WSAECONNREFUSED: s="Connection refused."; break;
	case WSAELOOP: s="Cannot translate name."; break;
	case WSAENAMETOOLONG: s="Name too long."; break;
	case WSAEHOSTDOWN: s="Host is down."; break;
	case WSAEHOSTUNREACH: s="No route to host."; break;
	case WSAENOTEMPTY: s="Directory not empty."; break;
	case WSAEPROCLIM: s="Too many processes."; break;
	case WSAEUSERS: s="User quota exceeded."; break;
	case WSAEDQUOT: s="Disk quota exceeded."; break;
	case WSAESTALE: s="Stale file handle reference."; break;
	case WSAEREMOTE: s="Item is remote."; break;
	case WSASYSNOTREADY: s="Network subsystem is unavailable."; break;
	case WSAVERNOTSUPPORTED: s="Winsock.dll version out of range."; break;
	case WSANOTINITIALISED: s="Successful WSAStartup not yet performed."; break;
	case WSAEDISCON: s="Graceful shutdown in progress."; break;
	case WSAENOMORE: s="No more results."; break;
	case WSAECANCELLED: s="Call has been canceled."; break;
	case WSAEINVALIDPROCTABLE: s="Procedure call table is invalid."; break;
	case WSAEINVALIDPROVIDER: s="Service provider is invalid."; break;
	case WSAEPROVIDERFAILEDINIT: s="Service provider failed to initialize."; break;
	case WSASYSCALLFAILURE: s="System call failure."; break;
	case WSASERVICE_NOT_FOUND: s="Service not found."; break;
	case WSATYPE_NOT_FOUND: s="Class type not found."; break;
	case WSA_E_NO_MORE: s="No more results."; break;
	case WSA_E_CANCELLED: s="Call was canceled."; break;
	case WSAEREFUSED: s="Database query was refused."; break;
	case WSAHOST_NOT_FOUND: s="Host not found."; break;
	case WSATRY_AGAIN: s="Nonauthoritative host not found."; break;
	case WSANO_RECOVERY: s="This is a nonrecoverable error."; break;
	case WSANO_DATA: s="Valid name, no data record of requested type."; break;

	default:
		s="Unknown error code";
		break;
	}

	s+=format(" (%i)",errnum);
	return s;
}




#endif // MRPT_OS_WINDOWS
