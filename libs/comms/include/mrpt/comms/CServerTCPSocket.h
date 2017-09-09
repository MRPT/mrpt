/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/utils/core_defs.h>
#include <mrpt/utils/COutputLogger.h>
#include <string>
#include <memory>  // unique_ptr
#include <mrpt/comms/link_pragmas.h>

namespace mrpt
{
namespace comms
{
class CClientTCPSocket;

/** A TCP socket that can be wait for client connections to enter.
  *  Unless otherwise noticed, operations are blocking.
 * \ingroup mrpt_comms_grp
  */
class COMMS_IMPEXP CServerTCPSocket : public utils::COutputLogger
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
		mrpt::utils::VerbosityLevel verbosityLevel = mrpt::utils::LVL_INFO);
	/** Dtor */
	virtual ~CServerTCPSocket();

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
#ifdef MRPT_OS_WINDOWS
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

}  // End of namespace
}  // End of namespace
