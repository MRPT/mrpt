/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers

#include <mrpt/utils/CClientTCPSocket.h>
#include <mrpt/utils/CMessage.h>
#include <mrpt/utils/net_utils.h>
#include <mrpt/system/os.h>
#include <cstring>

#ifdef MRPT_OS_WINDOWS
	// Windows
	#define _WINSOCK_DEPRECATED_NO_WARNINGS
	#include <winsock2.h>
	#include <winerror.h>
	#if defined(__BORLANDC__) || defined(_MSC_VER)
	#pragma comment (lib,"WS2_32.LIB")
	#endif
#else
	// Linux, Apple
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
	#include <netinet/tcp.h>
#endif

using namespace mrpt::utils;
using namespace mrpt::system;
using namespace mrpt;
using namespace std;


unsigned int CClientTCPSocket::DNS_LOOKUP_TIMEOUT_MS = 3000;

CClientTCPSocket::CClientTCPSocket( )
{
	MRPT_START

#ifdef MRPT_OS_WINDOWS
	// Init the WinSock Library:
	// ----------------------------
	WORD		wVersionRequested;
	WSADATA		wsaData;

	wVersionRequested = MAKEWORD( 2, 0 );

	if (WSAStartup( wVersionRequested, &wsaData ) )
		THROW_EXCEPTION("Error calling WSAStartup");

	m_hSock = INVALID_SOCKET;
#else
	// Linux, Apple
	m_hSock = -1;
#endif
	MRPT_END
}

CClientTCPSocket::~CClientTCPSocket( )
{
	// Close socket:
	close();
#ifdef MRPT_OS_WINDOWS
	WSACleanup();
#else
	// Nothing else to do.
#endif
}

void  CClientTCPSocket::close()
{
	MRPT_START

#ifdef MRPT_OS_WINDOWS
	// Delete socket:
	if (m_hSock != INVALID_SOCKET)
	{
		shutdown(m_hSock, 2 ); //SD_BOTH  );
		closesocket( m_hSock );
		m_hSock = INVALID_SOCKET;
	}
#else
	// Delete socket:
	if (m_hSock != -1)
	{
		shutdown(m_hSock, SHUT_RDWR  );
		::close( m_hSock );
		m_hSock = -1;
	}
#endif
	MRPT_END
}



/*---------------------------------------------------------------
						Read
 ---------------------------------------------------------------*/
size_t  CClientTCPSocket::Read(void *Buffer, size_t Count)
{
	MRPT_START

	return readAsync(Buffer,Count);

	MRPT_END
}

/*---------------------------------------------------------------
						Write
 ---------------------------------------------------------------*/
size_t  CClientTCPSocket::Write(const void *Buffer, size_t Count)
{
	MRPT_START

	return writeAsync(Buffer,Count);

	MRPT_END
}

/*---------------------------------------------------------------
						sendString
 ---------------------------------------------------------------*/
void  CClientTCPSocket::sendString( const std::string &str )
{
	Write( str.c_str(), str.size() );
}

/*---------------------------------------------------------------
						sendMessage
 ---------------------------------------------------------------*/
bool  CClientTCPSocket::sendMessage(
	const CMessage&	outMsg,
	const int timeout_ms
	)
{
	uint32_t	contentLen, toWrite,written;

	// --------------------------------
	// (1) Send a "magic word":
	// --------------------------------
	const char *magic= "MRPTMessage";
	toWrite = strlen(magic);

	written = writeAsync( magic, toWrite, timeout_ms );
	if (written!=toWrite) return false;	// Error!

	// --------------------------------
	// (2) Send the message type:
	// --------------------------------
	toWrite = sizeof( outMsg.type );

	written = writeAsync( &outMsg.type, toWrite, timeout_ms );
	if (written!=toWrite) return false;	// Error!

	// ---------------------------------------
	// (3) Send the message's content length:
	// ---------------------------------------
	contentLen = outMsg.content.size();
	toWrite = sizeof( contentLen );

	written = writeAsync( &contentLen, toWrite, timeout_ms );
	if (written!=toWrite) return false;	// Error!

	// ---------------------------------------
	// (4) Send the message's contents:
	// ---------------------------------------
	toWrite = contentLen;

	written = writeAsync( &outMsg.content[0], toWrite, timeout_ms );
	if (written!=toWrite) return false;	// Error!

	return true;
}

/*---------------------------------------------------------------
						receiveMessage
 ---------------------------------------------------------------*/
bool  CClientTCPSocket::receiveMessage(
	CMessage&			inMsg,
	unsigned int			timeoutStart_ms,
	unsigned int			timeoutBetween_ms
	)
{
	uint32_t	contentLen, toRead,actRead;

	// --------------------------------
	// (1) Read the "magic word":
	// --------------------------------
	char	magic[20]; // ;
	toRead = 11;

	actRead = readAsync( magic,toRead,timeoutStart_ms,timeoutBetween_ms );
	if (actRead!=toRead) return false;	// Error!
	magic[actRead] = 0; // Null-term string

	// Check magic:
	if ( 0 != os::_strcmpi( "MRPTMessage",magic ) )
		return false;

	// --------------------------------
	// (2) Read the message type:
	// --------------------------------
	toRead = sizeof( inMsg.type );

	actRead = readAsync( &inMsg.type,toRead,timeoutBetween_ms,timeoutBetween_ms );
	if (actRead!=toRead) return false;	// Error!

	// ---------------------------------------
	// (3) Read the message's content length:
	// ---------------------------------------
	toRead = sizeof( contentLen );

	actRead = readAsync( &contentLen,toRead,timeoutBetween_ms,timeoutBetween_ms );
	if (actRead!=toRead) return false;	// Error!

	// Reserve memory:
	inMsg.content.resize( contentLen );

	// ---------------------------------------
	// (4) Read the message's contents:
	// ---------------------------------------
	toRead = contentLen;

	actRead = readAsync( &inMsg.content[0],toRead,timeoutBetween_ms,timeoutBetween_ms);
	if (actRead!=toRead) return false;	// Error!

	return true;
}



/*---------------------------------------------------------------
						connect
 ---------------------------------------------------------------*/
void CClientTCPSocket::connect(
	const std::string	&remotePartAddress,
	unsigned short		remotePartTCPPort,
	unsigned int		timeout_ms )
{
	MRPT_START

	// Close existing socket, if any.
	if (m_hSock != INVALID_SOCKET)
		close();

	// Create the socket:
	if ( INVALID_SOCKET == (m_hSock = socket(AF_INET, SOCK_STREAM, 0)) )
		THROW_EXCEPTION( format("Error creating new client socket:\n%s",getLastErrorStr().c_str() ));

	struct sockaddr_in 		otherAddress;

	otherAddress.sin_family 	= AF_INET;
	otherAddress.sin_port 		= htons(remotePartTCPPort);

	// Resolve the IP address of the given host name
	std::string   solved_IP;
	if (!net::DNS_resolve_async(remotePartAddress,solved_IP,DNS_LOOKUP_TIMEOUT_MS))
		THROW_EXCEPTION_CUSTOM_MSG1("DNS lookup failed for '%s'",remotePartAddress.c_str());


	// Fill out from IP address text:
	otherAddress.sin_addr.s_addr = inet_addr(solved_IP.c_str());
	if (INADDR_NONE==otherAddress.sin_addr.s_addr)
		THROW_EXCEPTION_CUSTOM_MSG1("Invalid IP address provided: %s",solved_IP.c_str());

	// Set to NON-BLOCKING:
#ifdef MRPT_OS_WINDOWS
	unsigned long non_block_mode = 1;
	if (ioctlsocket(m_hSock, FIONBIO, &non_block_mode) ) THROW_EXCEPTION( "Error entering non-blocking mode with ioctlsocket()." );
#else
	int oldflags=fcntl(m_hSock, F_GETFL, 0);
	if (oldflags == -1)  THROW_EXCEPTION( "Error retrieving fcntl() of socket." );
	oldflags |= O_NONBLOCK;	  // Set NON-BLOCKING
	if (-1==fcntl(m_hSock, F_SETFL, oldflags))  THROW_EXCEPTION( "Error entering non-blocking mode with fcntl()." );
#endif

	// Try to connect:
	int r = ::connect( m_hSock , (struct sockaddr *)&otherAddress,sizeof(otherAddress));
#ifdef MRPT_OS_WINDOWS
	int er = WSAGetLastError();
	if (r < 0 && er != WSAEINPROGRESS && er != WSAEWOULDBLOCK)
#else
	int er = errno;
	if (r < 0 && er != EINPROGRESS)
#endif
		THROW_EXCEPTION( format("Error connecting to %s:%hu. Error: %s [%d]", remotePartAddress.c_str(), remotePartTCPPort, strerror(er),er));

	// Wait for connect:
	fd_set socket_set;
	timeval timer;

	FD_ZERO(&socket_set);
	FD_SET(m_hSock,&socket_set);

	timer.tv_sec  = timeout_ms/1000;
	timer.tv_usec = 1000*(timeout_ms%1000);

	int sel_ret = select(
		m_hSock+1,
		NULL,			// For read
		&socket_set,	// For write or *connect done*
		&socket_set,	// For errors
		timeout_ms==0 ? NULL : &timer
		);

	if (sel_ret == 0) THROW_EXCEPTION( format("Timeout connecting to '%s:%hu':\n%s",remotePartAddress.c_str(),remotePartTCPPort, getLastErrorStr().c_str() ));
	if (sel_ret ==-1) THROW_EXCEPTION( format("Error connecting to '%s:%hu':\n%s",remotePartAddress.c_str(),remotePartTCPPort, getLastErrorStr().c_str() ));

	// Now, make sure it was not an error!
	int valopt;
 #ifdef MRPT_OS_WINDOWS
	int lon = sizeof(int);
	getsockopt(m_hSock, SOL_SOCKET, SO_ERROR, (char*)(&valopt), &lon);
 #else
	socklen_t lon = sizeof(int);
	getsockopt(m_hSock, SOL_SOCKET, SO_ERROR, (void*)(&valopt), &lon);
 #endif

 #ifdef MRPT_OS_WINDOWS
	if (valopt) THROW_EXCEPTION( format("Error connecting to %s:%hu. Error: %i.", remotePartAddress.c_str(), remotePartTCPPort, valopt));
 #else
	if (valopt) THROW_EXCEPTION( format("Error connecting to %s:%hu. Error: %s.", remotePartAddress.c_str(), remotePartTCPPort, strerror(valopt)));
#endif
	// Connected!

	// If connected OK, remove the non-blocking flag:
#ifdef MRPT_OS_WINDOWS
	non_block_mode = 0;
	if (ioctlsocket(m_hSock, FIONBIO, &non_block_mode) ) THROW_EXCEPTION( "Error entering blocking mode with ioctlsocket()." );
#else
	oldflags &= ~O_NONBLOCK;	  // Set BLOCKING
	if (-1==fcntl(m_hSock, F_SETFL, oldflags))  THROW_EXCEPTION( "Error entering blocking mode with fcntl()." );
#endif

	// Save the IP of the other part.
	m_remotePartIP = remotePartAddress;

	MRPT_END
}

/*---------------------------------------------------------------
						isConnected
 ---------------------------------------------------------------*/
bool  CClientTCPSocket::isConnected()
{
	return (m_hSock != INVALID_SOCKET);
}



/*---------------------------------------------------------------
						readAsync
 ---------------------------------------------------------------*/
size_t  CClientTCPSocket::readAsync(
	void	*Buffer,
	const size_t	Count,
	const int		timeoutStart_ms,
	const int		timeoutBetween_ms )
{
	MRPT_START

	if (m_hSock == INVALID_SOCKET)   return 0;  // The socket is not connected!

	size_t			remainToRead, alreadyRead = 0;
	int				readNow;
	bool			timeoutExpired = false;

	struct timeval	timeoutSelect;
	struct timeval	*ptrTimeout;
	fd_set			sockArr;

    // Init fd_set structure & add our socket to it:
    FD_ZERO(&sockArr);
    FD_SET(m_hSock, &sockArr);

	// Loop until timeout expires or the socket is closed.
	while ( alreadyRead<Count && !timeoutExpired )
	{
		// Use the "first" or "between" timeouts:
		int curTimeout = alreadyRead==0 ? timeoutStart_ms : timeoutBetween_ms;

		if (curTimeout<0)
			ptrTimeout = NULL;
		else
		{
			timeoutSelect.tv_sec = curTimeout / 1000;
			timeoutSelect.tv_usec = 1000 * (curTimeout % 1000);
			ptrTimeout = &timeoutSelect;
		}

		// Wait for received data
		int selRet = ::select(
						 m_hSock+1,		// __nfds
						 &sockArr,		// Wait for read
						 NULL,			// Wait for write
						 NULL,			// Wait for except.
						 ptrTimeout);	// Timeout

		if( selRet==INVALID_SOCKET )
			THROW_EXCEPTION_CUSTOM_MSG1( "Error reading from socket: %s", getLastErrorStr().c_str() );

		if (selRet==0)
		{
			// Timeout:
			timeoutExpired = true;
		}
		else
		{
			// Compute remaining part:
			remainToRead = Count - alreadyRead;

			// Receive bytes:
			readNow = ::recv( m_hSock, ((char*)Buffer) + alreadyRead, (int)remainToRead, 0);

			if (readNow != INVALID_SOCKET)
			{
				// Accumulate the received length:
				alreadyRead += readNow;
			}
			else
			{
				// Error: Socket closed?
				this->close();
				return alreadyRead;
			}

			if (readNow==0 && remainToRead!=0)
			{
				// We had an event of data available, so if we have now a zero,
				//  the socket has been gracefully closed:
				timeoutExpired = true;
				close();
			}
		}
	} // end while

	return alreadyRead;

	MRPT_END
}

/*---------------------------------------------------------------
						writeAsync
 ---------------------------------------------------------------*/
size_t  CClientTCPSocket::writeAsync(
	const void	*Buffer,
	const size_t	Count,
	const int		timeout_ms )
{
	MRPT_START

	if (m_hSock == INVALID_SOCKET)  return 0;		// The socket is not connected!

	size_t		remainToWrite, alreadyWritten = 0;
	int			writtenNow;
	bool		timeoutExpired = false;

	struct timeval	timeoutSelect;
	struct timeval	*ptrTimeout;
	fd_set			sockArr;

    // Init fd_set structure & add our socket to it:
    FD_ZERO(&sockArr);
    FD_SET(m_hSock, &sockArr);

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

	// Loop until timeout expires or the socket is closed.
	while ( alreadyWritten<Count && !timeoutExpired )
	{
		// Wait for received data
		int selRet = ::select(
						 m_hSock+1,		// __nfds
						 NULL,			// Wait for read
						 &sockArr,		// Wait for write
						 NULL,			// Wait for except.
						 ptrTimeout);	// Timeout

		if( selRet==INVALID_SOCKET )
			THROW_EXCEPTION_CUSTOM_MSG1( "Error writing to socket: %s", getLastErrorStr().c_str() );

		if (selRet==0)
		{
			// Timeout:
			timeoutExpired = true;
		}
		else
		{
			// We have room to write data!

			// Compute remaining part:
			remainToWrite = Count - alreadyWritten;

			// Receive bytes:
			writtenNow = ::send( m_hSock, ((char*)Buffer) + alreadyWritten, (int)remainToWrite, 0);

			if (writtenNow != INVALID_SOCKET)
			{
				// Accumulate the received length:
				alreadyWritten += writtenNow;
			}
		}

	} // end while

	return alreadyWritten;

	MRPT_END
}



/*---------------------------------------------------------------
						getReadPendingBytes
 ---------------------------------------------------------------*/
size_t  CClientTCPSocket::getReadPendingBytes()
{
	if (m_hSock == INVALID_SOCKET)   return 0;  // The socket is not connected!
	unsigned long ret=0;
	if (
#ifdef MRPT_OS_WINDOWS
	ioctlsocket(m_hSock,FIONREAD,&ret)
#else
	ioctl(m_hSock, FIONREAD, &ret)
#endif
	)
	{
		  THROW_EXCEPTION( "Error invoking ioctlsocket(FIONREAD)" )
	}
	else  return ret;
}


/*---------------------------------------------------------------
						setTCPNoDelay
 ---------------------------------------------------------------*/
int CClientTCPSocket::setTCPNoDelay( const int &newValue )
{
	int length = sizeof( newValue );

	return setsockopt(m_hSock,IPPROTO_TCP,TCP_NODELAY,(char*)&newValue,length);
}


/*---------------------------------------------------------------
						getTCPNoDelay
 ---------------------------------------------------------------*/
int CClientTCPSocket::getTCPNoDelay()
{
	int value;
#ifdef MRPT_OS_WINDOWS
	int length = sizeof ( value );
#else
	unsigned int length = sizeof ( value );
#endif
	int res = getsockopt(m_hSock, IPPROTO_TCP, TCP_NODELAY, (char*)&value, &length );

	if (res==-1)
	     return -1;
	else return value;
}


/*---------------------------------------------------------------
						setSOSendBufffer
 ---------------------------------------------------------------*/
int CClientTCPSocket::setSOSendBufffer( const int &newValue )
{
	const unsigned int length = sizeof( newValue );

	return setsockopt(m_hSock, SOL_SOCKET, SO_SNDBUF, (char*)&newValue, length );
}


/*---------------------------------------------------------------
						getSOSendBufffer
 ---------------------------------------------------------------*/
int CClientTCPSocket::getSOSendBufffer()
{
	int value;
#ifdef MRPT_OS_WINDOWS
	int length = sizeof ( value );
#else
	unsigned int length = sizeof ( value );
#endif
	getsockopt(m_hSock, SOL_SOCKET, SO_SNDBUF, (char*)&value, &length );

	return value;
}
