/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/core/common.h>
#include <mrpt/system/COutputLogger.h>
#include <string>
#include <memory>  // unique_ptr

namespace mrpt::comms
{
class CClientTCPSocket;

/** A TCP socket that can be wait for client connections to enter.
 *  Unless otherwise noticed, operations are blocking.
 * \ingroup mrpt_comms_grp
 */
class CServerTCPSocket : public mrpt::system::COutputLogger
{
   public:
	/** Constructor that creates the socket, performs binding, and start
	 * listening mode.
	 *  \param listenPort The port to bound to.
	 *  \param IPaddress The interface to bound the socket to. By default is
	 * 127.0.0.1 for localhost, for all network interfaces use 0.0.0.0.
	 *  \param maxConnectionsWaiting Maximum number of incoming connections
	 * waiting for "accept" before new ones are rejected.
	 *  You can check if the socket has been created OK with "isListening".
	 * \sa isListening
	 * \exception std::exception If there is any error creating the socket,
	 * with a textual description of the error.
	 */
	CServerTCPSocket(
		unsigned short listenPort,
		const std::string& IPaddress = std::string("127.0.0.1"),
		int maxConnectionsWaiting = 50,
		mrpt::system::VerbosityLevel verbosityLevel = mrpt::system::LVL_INFO);
	/** Dtor */
	~CServerTCPSocket() override;

	/** Returns true if the socket was successfully open and it's bound to the
	 * desired port. */
	bool isListening();

	/** Waits for an incoming connection (indefinitely, or with a given timeout)
	 * The returned object represents the new connection, and MUST BE deleted
	 * by the user when no longer needed.
	 * \param timeout_ms The timeout for the waiting, in milliseconds. Set this
	 * to "-1" to disable timeout (i.e. timeout=infinite)
	 * \return The incoming connection, or nullptr on timeout or error.
	 */
	std::unique_ptr<CClientTCPSocket> accept(int timeout_ms = -1);

   private:
/** The handle for the listening server TCP socket. */
#ifdef _WIN32
	unsigned int
#else
	int
#endif
		m_serverSock;

	/** Returns a description of the last Sockets error */
	std::string getLastErrorStr();

	/** Common code called from the platform-dependant constructor */
	void setupSocket(
		unsigned short listenPort, const std::string& IPaddress,
		int maxConnectionsWaiting);

};  // End of class def.

}  // namespace mrpt::comms
