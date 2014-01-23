/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
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
