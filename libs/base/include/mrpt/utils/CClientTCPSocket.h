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
#ifndef  CClientTCPSocket_H
#define  CClientTCPSocket_H

#include <mrpt/config.h>
#include <mrpt/utils/utils_defs.h>
#include <mrpt/system/os.h>
#include <mrpt/utils/CStream.h>

namespace mrpt
{
namespace utils
{
	class CServerTCPSocket;
	class CMessage;

	/** \defgroup network_grp Networking, sockets, DNS
	  * \ingroup mrpt_base_grp */

	/** A TCP socket that can be connected to a TCP server, implementing MRPT's CStream interface for passing objects as well as generic read/write methods.
	  *  Unless otherwise noticed, operations are blocking.
	  *
	  *  Note that for convenience, DNS lookup is performed with a timeout (default=3000ms), which can be changed by the static member CClientTCPSocket::DNS_LOOKUP_TIMEOUT_MS
	  * \ingroup network_grp
	  */
	class BASE_IMPEXP CClientTCPSocket : public CStream
	{
		friend class CServerTCPSocket;

	public:
		/** See description of CClientTCPSocket */
		static unsigned int DNS_LOOKUP_TIMEOUT_MS;

	protected:

#ifdef MRPT_OS_WINDOWS
		/** The handle for the connected TCP socket, or INVALID_SOCKET
		  */
#	if MRPT_WORD_SIZE==64
		uint64_t	m_hSock;
#	else
		uint32_t	m_hSock;
#	endif
#else

		/** The handle for the connected TCP socket, or -1
		  */
		int				m_hSock;
#endif

		/** The IP address of the remote part of the connection.
		  */
		std::string		m_remotePartIP;

		/** The TCP port of the remote part of the connection.
		  */
		unsigned short	m_remotePartPort;


		/** Introduces a virtual method responsible for reading from the stream (This method BLOCKS)
		  * This method is implemented as a call to "readAsync" with infinite timeouts.
		  * \sa readAsync
		  */
		size_t  Read(void *Buffer, size_t Count);

		/** Introduces a virtual method responsible for writing to the stream.
		  *  Write attempts to write up to Count bytes to Buffer, and returns the number of bytes actually written.
		  *  This method is implemented as a call to "writeAsync" with infinite timeouts.
		  * \sa writeAsync
		 */
		size_t  Write(const void *Buffer, size_t Count);

		/** Returns a description of the last error */
		std::string  getLastErrorStr();

	public:
		/** Default constructor
		  * \sa connect
		  */
		CClientTCPSocket( );

		/** Destructor
		 */
		~CClientTCPSocket( );

		/** Establishes a connection with a remote part.
		  * \param remotePartAddress This string can be a host name, like "server" or "www.mydomain.org", or an IP address "11.22.33.44".
		  * \param remotePartTCPPort The port on the remote machine to connect to.
		  * \param timeout_ms  The timeout to wait for the connection (0: NO TIMEOUT)
		  * \exception This method raises an exception if an error is found with a textual description of the error.
		  */
		void connect(
			const std::string	&remotePartAddress,
			unsigned short		remotePartTCPPort,
			unsigned int		timeout_ms = 0 );

		/** Returns true if this objects represents a successfully connected socket.
		  */
		bool  isConnected();

		/** Closes the connection.
		  */
		void  close();

		/** Writes a string to the socket.
		  * \exception std::exception On communication errors
		  */
		void  sendString( const std::string &str );

		/** This virtual method has no effect in this implementation over a TCP socket, and its use raises an exception
		 */
		uint64_t Seek(long Offset, CStream::TSeekOrigin Origin = sFromBeginning)
		{
		    MRPT_START
			MRPT_UNUSED_PARAM(Offset); MRPT_UNUSED_PARAM(Origin);
			THROW_EXCEPTION("This method has no effect in this class!");
		    MRPT_END
		}

		/** This virtual method has no effect in this implementation over a TCP socket, and its use raises an exception
		 */
		uint64_t getTotalBytesCount()
		{
		    MRPT_START
			THROW_EXCEPTION("This method has no effect in this class!");
		    MRPT_END
		}

		/** This virtual method has no effect in this implementation over a TCP socket, and its use raises an exception
		 */
		uint64_t getPosition()
		{
		    MRPT_START
			THROW_EXCEPTION("This method has no effect in this class!");
		    MRPT_END
		}

		/** A method for reading from the socket with an optional timeout.
		  * \param Buffer The destination of data.
		  * \param Cound The number of bytes to read.
		  * \param timeoutStart_ms The maximum timeout (in milliseconds) to wait for the starting of data from the other side.
		  * \param timeoutBetween_ms The maximum timeout (in milliseconds) to wait for a chunk of data after a previous one.
		  *  Set timeout's to -1 to block until the desired number of bytes are read, or an error happens.
		  *  \return The number of actually read bytes.
		  */
		size_t  readAsync(
			void	*Buffer,
			const size_t	Count,
			const int	timeoutStart_ms = -1,
			const int	timeoutBetween_ms = -1);

		/** A method for writing to the socket with optional timeouts.
		  *  The method supports writing block by block as the socket allows us to write more data.
		  * \param Buffer The data.
		  * \param Cound The number of bytes to write.
		  * \param timeout_ms The maximum timeout (in milliseconds) to wait for the socket to be available for writing (for each block).
		  *  Set timeout's to -1 to block until the desired number of bytes are written, or an error happens.
		  *  \return The number of actually written bytes.
		  */
		size_t  writeAsync(
			const void	*Buffer,
			const size_t Count,
			const int	timeout_ms = -1 );

		/** Send a message through the TCP stream.
		  * \param outMsg The message to be shown.
		  * \param timeout_ms The maximum timeout (in milliseconds) to wait for the socket in each write operation.
		  * \return Returns false on any error, or true if everything goes fine.
		  */
		bool  sendMessage(
			const CMessage&	outMsg,
			const int timeout_ms = -1
			);

		/** Waits for an incoming message through the TCP stream.
		  * \param inMsg The received message is placed here.
		  * \param timeoutStart_ms The maximum timeout (in milliseconds) to wait for the starting of data from the other side.
		  * \param timeoutBetween_ms The maximum timeout (in milliseconds) to wait for a chunk of data after a previous one.
		  * \return Returns false on any error (or timeout), or true if everything goes fine.
		  */
		bool  receiveMessage(
			CMessage&			inMsg,
			const unsigned int	timeoutStart_ms = 100,
			const unsigned int	timeoutBetween_ms = 1000
			);

		/** Return the number of bytes already in the receive queue (they can be read without waiting) */
		size_t  getReadPendingBytes();

	}; // End of class def.

	} // End of namespace
} // end of namespace
#endif
