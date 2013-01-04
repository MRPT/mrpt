/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
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
#ifndef  CServerTCPSocket_H
#define  CServerTCPSocket_H

#include <mrpt/config.h>
#include <mrpt/utils/utils_defs.h>
#include <mrpt/utils/CStream.h>

#include <mrpt/utils/CDebugOutputCapable.h>

namespace mrpt
{
namespace utils
{
	class CClientTCPSocket;

	/** A TCP socket that can be wait for client connections to enter.
	  *  Unless otherwise noticed, operations are blocking.
	 * \ingroup network_grp
	  */
	class BASE_IMPEXP CServerTCPSocket : public utils::CDebugOutputCapable
	{
	private:

#ifdef MRPT_OS_WINDOWS
		/** The handle for the listening server TCP socket.
		  */
		unsigned int	m_serverSock;
#else
		/** The handle for the listening server TCP socket.
		  */
		int				m_serverSock;
#endif

		/** Returns a description of the last error */
		std::string  getLastErrorStr();

		bool		m_verbose;

		/** Common code called from the platform-dependant constructor */
		void setupSocket(
			unsigned short	  listenPort,
			const std::string &IPaddress,
			int               maxConnectionsWaiting );

	public:
		/** Constructor that creates the socket, performs binding, and start listening mode.
		  *  \param listenPort The port to bound to.
		  *  \param IPaddress The interface to bound the socket to. By default 127.0.0.1 implies listening at all network interfaces.
		  *  \param maxConnectionsWaiting Maximum number of incoming connections waiting for "accept" before new ones are rejected.
		  *  \param verbose Whether to dump state information to the output defined in the utils::CDebugOutputCapable interface.
		  *  You can check if the socket has been created OK with "isListening".
		  * \sa isListening
		  * \exception std::exception If there is any error creating the socket, with a textual description of the error.
		  */
		CServerTCPSocket(
			unsigned short	  listenPort,
			const std::string &IPaddress = std::string("127.0.0.1"),
			int               maxConnectionsWaiting = 50,
			bool			verbose = false
			);

		/** Destructor
		 */
		~CServerTCPSocket( );

		/** Returns true if the socket was successfully open and it's bound to the desired port.
		  */
		bool isListening();

		/** Waits for an incoming connection (indefinitely, or with a given timeout)
		  * The returned object represents the new connection, and MUST BE deleted by the user when no longer needed.
		  * \param timeout_ms The timeout for the waiting, in milliseconds. Set this to "-1" to disable timeout (i.e. timeout=infinite)
		  * \return The incoming connection, or NULL on timeout or error.
		  */
		CClientTCPSocket *  accept( int timeout_ms = -1 );



	}; // End of class def.

} // End of namespace
} // End of namespace

#endif  // file
