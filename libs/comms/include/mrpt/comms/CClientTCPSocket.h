/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/config.h>  // MRPT_WORD_SIZE
#include <mrpt/io/CStream.h>
#include <mrpt/system/os.h>
#include <cstdint>
#include <cstring>  // strlen()
#include <string>

namespace mrpt
{
/** Serial and networking devices and utilities */
namespace comms
{
class CServerTCPSocket;

/** A TCP socket that can be connected to a TCP server, implementing MRPT's
 * CStream interface for passing objects as well as generic read/write methods.
 *  Unless otherwise noticed, operations are blocking.
 *
 *  Note that for convenience, DNS lookup is performed with a timeout
 * (default=3000ms), which can be changed by the static member
 * CClientTCPSocket::DNS_LOOKUP_TIMEOUT_MS
 * \ingroup mrpt_comms_grp
 */
class CClientTCPSocket : public mrpt::io::CStream
{
	friend class CServerTCPSocket;

   public:
	/** See description of CClientTCPSocket */
	static unsigned int DNS_LOOKUP_TIMEOUT_MS;

   protected:
#ifdef _WIN32
/** The handle for the connected TCP socket, or INVALID_SOCKET
 */
#if MRPT_WORD_SIZE == 64
	uint64_t m_hSock;
#else
	uint32_t m_hSock;
#endif
#else
	/** The handle for the connected TCP socket, or -1 */
	int m_hSock;
#endif
	/** The IP address of the remote part of the connection. */
	std::string m_remotePartIP;
	/** The TCP port of the remote part of the connection. */
	unsigned short m_remotePartPort = 0;

	/** Introduces a virtual method responsible for reading from the stream
	 * (This method BLOCKS)
	 * This method is implemented as a call to "readAsync" with infinite
	 * timeouts.
	 * \sa readAsync */
	size_t Read(void* Buffer, size_t Count) override;

	/** Introduces a virtual method responsible for writing to the stream.
	 *  Write attempts to write up to Count bytes to Buffer, and returns the
	 * number of bytes actually written.
	 *  This method is implemented as a call to "writeAsync" with infinite
	 * timeouts.
	 * \sa writeAsync */
	size_t Write(const void* Buffer, size_t Count) override;

	/** Returns a description of the last Sockets error */
	std::string getLastErrorStr();

   public:
	/** Default constructor \sa connect  */
	CClientTCPSocket();

	/** Destructor */
	~CClientTCPSocket() override;

	/** Establishes a connection with a remote part.
	 * \param remotePartAddress This string can be a host name, like "server"
	 * or "www.mydomain.org", or an IP address "11.22.33.44".
	 * \param remotePartTCPPort The port on the remote machine to connect to.
	 * \param timeout_ms  The timeout to wait for the connection (0: NO
	 * TIMEOUT)
	 * \exception This method raises an exception if an error is found with a
	 * textual description of the error.
	 */
	void connect(
		const std::string& remotePartAddress, unsigned short remotePartTCPPort,
		unsigned int timeout_ms = 0);

	/** Returns true if this objects represents a successfully connected socket
	 */
	bool isConnected();

	/** Closes the connection */
	void close();

	/** Writes a string to the socket.
	 * \exception std::exception On communication errors
	 */
	void sendString(const std::string& str);

	/** This virtual method has no effect in this implementation over a TCP
	 * socket, and its use raises an exception */
	uint64_t Seek(
		int64_t off, CStream::TSeekOrigin org = sFromBeginning) override;

	/** This virtual method has no effect in this implementation over a TCP
	 * socket, and its use raises an exception */
	uint64_t getTotalBytesCount() const override;

	/** This virtual method has no effect in this implementation over a TCP
	 * socket, and its use raises an exception */
	uint64_t getPosition() const override;

	/** A method for reading from the socket with an optional timeout.
	 * \param Buffer The destination of data.
	 * \param Cound The number of bytes to read.
	 * \param timeoutStart_ms The maximum timeout (in milliseconds) to wait for
	 * the starting of data from the other side.
	 * \param timeoutBetween_ms The maximum timeout (in milliseconds) to wait
	 * for a chunk of data after a previous one.
	 *  Set timeout's to -1 to block until the desired number of bytes are
	 * read, or an error happens.
	 *  \return The number of actually read bytes.
	 */
	size_t readAsync(
		void* Buffer, const size_t Count, const int timeoutStart_ms = -1,
		const int timeoutBetween_ms = -1);

	/** A method for writing to the socket with optional timeouts.
	 *  The method supports writing block by block as the socket allows us to
	 * write more data.
	 * \param Buffer The data.
	 * \param Cound The number of bytes to write.
	 * \param timeout_ms The maximum timeout (in milliseconds) to wait for the
	 * socket to be available for writing (for each block).
	 *  Set timeout's to -1 to block until the desired number of bytes are
	 * written, or an error happens.
	 *  \return The number of actually written bytes.
	 */
	size_t writeAsync(
		const void* Buffer, const size_t Count, const int timeout_ms = -1);

	/** Send a message through the TCP stream.
	 * \param outMsg The message to be shown.
	 * \param timeout_ms The maximum timeout (in milliseconds) to wait for the
	 * socket in each write operation.
	 * \return Returns false on any error, or true if everything goes fine.
	 * \tparam MESSAGE can be mrpt::serialization::CMessage
	 */
	template <class MESSAGE>
	bool sendMessage(const MESSAGE& outMsg, const int timeout_ms = -1)
	{
		// (1) Send a "magic word":
		const char* magic = "MRPTMessage";
		uint32_t toWrite = strlen(magic);
		uint32_t written = writeAsync(magic, toWrite, timeout_ms);
		if (written != toWrite) return false;  // Error!
		// (2) Send the message type:
		toWrite = sizeof(outMsg.type);
		written = writeAsync(&outMsg.type, toWrite, timeout_ms);
		if (written != toWrite) return false;  // Error!
		// (3) Send the message's content length:
		uint32_t contentLen = outMsg.content.size();
		toWrite = sizeof(contentLen);
		written = writeAsync(&contentLen, toWrite, timeout_ms);
		if (written != toWrite) return false;  // Error!
		// (4) Send the message's contents:
		toWrite = contentLen;
		written = writeAsync(&outMsg.content[0], toWrite, timeout_ms);
		if (written != toWrite) return false;  // Error!
		return true;
	}

	/** Waits for an incoming message through the TCP stream.
	 * \param inMsg The received message is placed here.
	 * \param timeoutStart_ms The maximum timeout (in milliseconds) to wait for
	 * the starting of data from the other side.
	 * \param timeoutBetween_ms The maximum timeout (in milliseconds) to wait
	 * for a chunk of data after a previous one.
	 * \return Returns false on any error (or timeout), or true if everything
	 * goes fine.
	 * \tparam MESSAGE can be mrpt::serialization::CMessage
	 */
	template <class MESSAGE>
	bool receiveMessage(
		MESSAGE& inMsg, const unsigned int timeoutStart_ms = 100,
		const unsigned int timeoutBetween_ms = 1000)
	{
		// (1) Read the "magic word":
		char magic[20];
		uint32_t toRead = 11;
		uint32_t actRead =
			readAsync(magic, toRead, timeoutStart_ms, timeoutBetween_ms);
		if (actRead != toRead) return false;  // Error!
		magic[actRead] = 0;  // Null-term string
		// Check magic:
		if (0 != ::strcmp("MRPTMessage", magic)) return false;
		// (2) Read the message type:
		toRead = sizeof(inMsg.type);
		actRead = readAsync(
			&inMsg.type, toRead, timeoutBetween_ms, timeoutBetween_ms);
		if (actRead != toRead) return false;  // Error!
		// (3) Read the message's content length:
		uint32_t contentLen;
		toRead = sizeof(contentLen);
		actRead = readAsync(
			&contentLen, toRead, timeoutBetween_ms, timeoutBetween_ms);
		if (actRead != toRead) return false;  // Error!
		inMsg.content.resize(contentLen);
		// (4) Read the message's contents:
		toRead = contentLen;
		actRead = readAsync(
			&inMsg.content[0], toRead, timeoutBetween_ms, timeoutBetween_ms);
		if (actRead != toRead) return false;  // Error!
		return true;
	}

	/** Return the number of bytes already in the receive queue (they can be
	 * read without waiting) */
	size_t getReadPendingBytes();

	/** Set the TCP no delay option of the protocol (Nagle algorithm).
	 * \param newValue New value (0 enable Nagle algorithm, 1 disable).
	 * \return Return a number lower than 0 if any error occurred.
	 */
	int setTCPNoDelay(const int& newValue);

	/** Return the value of the TCPNoDelay option. */
	int getTCPNoDelay();

	/** Set the size of the SO send buffer. This buffer is used to store data,
	 * and is sended when is full.
	 * \param newValue New size of the SO send buffer.
	 * \return Return a number lower than 0 if any error occurred.
	 */
	int setSOSendBufffer(const int& newValue);

	/** Return the current size of the SO send buffer. */
	int getSOSendBufffer();

};  // End of class def.

}  // namespace comms
}  // namespace mrpt
