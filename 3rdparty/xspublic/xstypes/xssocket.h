
//  Copyright (c) 2003-2019 Xsens Technologies B.V. or subsidiaries worldwide.
//  All rights reserved.
//  
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//  
//  1.	Redistributions of source code must retain the above copyright notice,
//  	this list of conditions, and the following disclaimer.
//  
//  2.	Redistributions in binary form must reproduce the above copyright notice,
//  	this list of conditions, and the following disclaimer in the documentation
//  	and/or other materials provided with the distribution.
//  
//  3.	Neither the names of the copyright holders nor the names of their contributors
//  	may be used to endorse or promote products derived from this software without
//  	specific prior written permission.
//  
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
//  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
//  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
//  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
//  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS 
//  SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES 
//  OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE 
//  ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.
//  

#ifndef XSSOCKET_H
#define XSSOCKET_H

#include "xsstring.h"
#include "xsbytearray.h"
#include "xsresultvalue.h"
#include "xstypedefs.h"

struct XsSocket;
struct XsSocketInterface;
typedef struct XsSocket XsSocket;

/*! \brief the protocol on top of IP */
enum IpProtocol {
	IP_UDP, /*!< \brief The UDP/IP protocol */
	IP_TCP  /*!< \brief The TCP/IP protocol */
};

/*! \brief The network layer protocol, or IP address family */
enum NetworkLayerProtocol {
	NLP_IPV4, /*!< \brief IPv4 address family */
	NLP_IPV6, /*!< \brief IPv6 address family */
	NLP_IPVX  /*!< \brief any IP address family */
};

enum XsSocketOption {
	XSO_ReuseAddress,	/*!< \brief Allow bind to reuse a local port (using different local addresses) */
	XSO_ReusePort		/*!< \brief Allow bind to reuse the exact local address */
};

#ifdef WIN32
#	if defined(_WIN64)
	typedef unsigned __int64 XSOCKET;
#	else
	typedef unsigned int XSOCKET;
#	endif
#else
typedef int XSOCKET;
#endif

#ifdef __cplusplus
extern "C" {
#endif
#ifndef __cplusplus
#define XSSOCKET_INITIALIZER { NULL }
#endif

#define XSSOCKET_INFINITE_TIMEOUT -1

XSTYPES_DLL_API void XsSocket_create(XsSocket* thisPtr, enum NetworkLayerProtocol ip, enum IpProtocol protocol);
XSTYPES_DLL_API void XsSocket_createFromFileDescriptor(XsSocket* thisPtr, int sockfd, XsDataFlags flags);
XSTYPES_DLL_API void XsSocket_destroy(XsSocket* thisPtr);
XSTYPES_DLL_API XsResultValue XsSocket_close(XsSocket* thisPtr);

XSTYPES_DLL_API XSOCKET XsSocket_nativeDescriptor(XsSocket const* thisPtr);

XSTYPES_DLL_API int XsSocket_select(XsSocket* thisPtr, int mstimeout, int *canRead, int *canWrite);
XSTYPES_DLL_API int XsSocket_read(XsSocket* thisPtr, void* dest, XsSize size, int timeout);
XSTYPES_DLL_API int XsSocket_readFrom(XsSocket* thisPtr, void* dest, XsSize size, XsString* hostname, uint16_t* port, int timeout);
XSTYPES_DLL_API int XsSocket_read2ByteArray(XsSocket* thisPtr, XsByteArray* dest, int timeout);
XSTYPES_DLL_API int XsSocket_readFrom2ByteArray(XsSocket* thisPtr, XsByteArray* dest, XsString* hostname, uint16_t* port, int timeout);

XSTYPES_DLL_API int XsSocket_write(XsSocket* thisPtr, const void* buffer, XsSize size);
XSTYPES_DLL_API int XsSocket_writeTo(XsSocket* thisPtr, const void* buffer, XsSize size, const XsString* hostname, uint16_t port);

XSTYPES_DLL_API void XsSocket_flush(XsSocket* thisPtr);

XSTYPES_DLL_API XsSocket* XsSocket_accept(XsSocket* thisPtr, int mstimeout);

XSTYPES_DLL_API XsSocket* XsSocket_allocate();
XSTYPES_DLL_API void XsSocket_free(XsSocket* thisPtr);

XSTYPES_DLL_API XsResultValue XsSocket_setSocketOption(XsSocket* thisPtr, enum XsSocketOption option, void* valuePtr, int valueSize);
XSTYPES_DLL_API XsResultValue XsSocket_bind(XsSocket* thisPtr, const XsString* hostname, uint16_t port);
XSTYPES_DLL_API XsResultValue XsSocket_listen(XsSocket* thisPtr, int maxPending);
XSTYPES_DLL_API XsResultValue XsSocket_connect(XsSocket* thisPtr, const XsString* host, uint16_t port);

XSTYPES_DLL_API int XsSocket_isUsable(const XsSocket* thisPtr);
XSTYPES_DLL_API void XsSocket_getRemoteAddress(const XsSocket* thisPtr, XsString *address);
XSTYPES_DLL_API int XsSocket_getLastSystemError(const XsSocket* thisPtr);
XSTYPES_DLL_API int XsSocket_enableBroadcasts(XsSocket* thisPtr, int enable);
XSTYPES_DLL_API int XsSocket_broadcast(XsSocket* thisPtr, const void* buffer, XsSize size, uint16_t port);

#ifdef __cplusplus
}
#endif

/*! \brief A platform independent socket implementation

	The internals are hidden to remove any inconvenience that
	may occur because of the order of inclusion of system headers.
*/
struct XsSocket
{
#ifdef __cplusplus
	/*! \brief \copybrief XsSocket_create

		\param[in] protocol the protocol on top of IP to use
		\param[in] ipVersion the ip address family to use
	*/
	inline explicit XsSocket(IpProtocol protocol, NetworkLayerProtocol ipVersion = NLP_IPVX)
		: d(NULL)
	{
		(void)d;
		XsSocket_create(this, ipVersion, protocol);
	}

	/*! \brief \copybrief XsSocket_createFromFileDescriptor

		\param[in] sockfd the file descriptor of the socket to wrap
		\param[in] flags if set to XSDF_Managed the wrapped socket will be closed when object is destroyed
	*/
	inline explicit XsSocket(int sockfd, XsDataFlags flags = XSDF_None)
		: d(NULL)
	{
		(void)d;
		XsSocket_createFromFileDescriptor(this, sockfd, flags);
	}

	/*! \brief \copybrief XsSocket_destroy
	*/
	inline ~XsSocket()
	{
		XsSocket_destroy(this);
	}

	/*! \brief \copybrief XsSocket_close
	*/
	inline XsResultValue close()
	{
		return XsSocket_close(this);
	}

	/*! \brief Return the native file descriptor */
	inline XSOCKET nativeDescriptor() const
	{
		return XsSocket_nativeDescriptor(this);
	}

	/*! \brief read data from the socket into \a buffer

		\param[in,out] buffer the buffer to fill, contents are automatically adjusted to the data size available
		\param[in] timeout the maximum time to wait before data is in in milliseconds

		\returns the number of bytes read from the socket
		\see read(void*,XsSize,int)
		\see XsSocket_read
	*/
	inline int read(XsByteArray& buffer, int timeout = 0)
	{
		return XsSocket_read2ByteArray(this, &buffer, timeout);
	}

	/*! \brief \copybrief XsSocket_read

		\param[in,out] dest the destination buffer for the incoming data
		\param[in] size the size of the destination buffer
		\param[in] timeout the maximum time to wait before data is in in milliseconds

		\returns the number of bytes read from the socket
		\see read(XsByteArray&,int)
		\see XsSocket_read
	*/
	inline int read(void* dest, XsSize size, int timeout = 0)
	{
		return XsSocket_read(this, dest, size, timeout);
	}

	/*! \brief read data from the socket into \a buffer

		\param[in,out] buffer the buffer to write into
		\param[in,out] hostname a pointer to a string that contains the hostname of the sender after returning from this function. May be NULL
		\param[in,out] port a pointer that will contain the port of the sender after returning from this function. May be NULL
		\param[in] timeout the read timeout in milliseconds

		\returns the size of the data read from the socket, -1 on error
		\see readFrom(void*,XsSize,XsString*,uint16_t*,int)
		\see XsSocket_readFrom
	*/
	inline int readFrom(XsByteArray& buffer, XsString* hostname = NULL, uint16_t* port = NULL, int timeout = 0)
	{
		return XsSocket_readFrom2ByteArray(this, &buffer, hostname, port, timeout);
	}

	/*! \brief \copybrief XsSocket_readFrom

		\param[in,out] dest the buffer to write into
		\param[in] size the size of the destination buffer
		\param[in,out] hostname a pointer to a string that contains the hostname of the sender after returning from this function. May be NULL
		\param[in,out] port a pointer that will contain the port of the sender after returning from this function. May be NULL
		\param[in] timeout the read timeout in milliseconds

		\returns the size of the data read from the socket, -1 on error
		\see readFrom(XsByteArray,XsString*,uint16_t,int)
		\see XsSocket_readFrom
	*/
	inline int readFrom(void* dest, XsSize size, XsString* hostname = NULL, uint16_t* port = NULL, int timeout = 0)
	{
		return XsSocket_readFrom(this, dest, size, hostname, port, timeout);
	}

	/*! \brief Write the data in \a buffer to the socket

		\param[in] buffer the data to write

		\returns the number of bytes written
		\see write(const void*,XsSize)
		\see XsSocket_write
	*/
	int write(const XsByteArray& buffer)
	{
		return write(buffer.data(), buffer.size());
	}

	/*! \brief \copybrief XsSocket_write

		\param[in] data the data to write
		\param[in] size the size of the data to write

		\returns the number of bytes written to the socket
		\see write(const XsByteArray&)
		\see XsSocket_write
	*/
	int write(const void* data, XsSize size)
	{
		return XsSocket_write(this, data, size);
	}

	/*! \brief Write the data in \a buffer to the socket

		\param[in] buffer the data to write
		\param[in] hostname the host to write data to
		\param[in] port the port to write data to

		\returns the number of bytes written
		\see writeTo(const void*,XsSize,XsString,uint16_t)
		\see XsSocket_writeTo
	*/
	int writeTo(const XsByteArray& buffer, const XsString& hostname, uint16_t port)
	{
		return writeTo(buffer.data(), buffer.size(), hostname, port);
	}

	/*! \brief \copybrief XsSocket_writeTo

		\param[in] data the data to write
		\param[in] size the size of the data to write
		\param[in] hostname the host to write data to
		\param[in] port the port to write data to

		\returns the number of bytes written to the socket
		\see writeTo(XsByteArray,XsString,uint16_t)
		\see XsSocket_writeTo
	*/
	int writeTo(const void* data, XsSize size, const XsString& hostname, uint16_t port)
	{
		return XsSocket_writeTo(this, data, size, &hostname, port);
	}

	/*! \brief \copybrief XsSocket_flush
	 */
	void flush()
	{
		XsSocket_flush(this);
	}

	/*! \brief \copybrief XsSocket_select
		\param[in] mstimeout timeout to wait for socket to be readable/writeable
		\param[in,out] canRead if not null then will be set to non-zero to indicate that the socket has data to read.
		\param[in,out] canWrite if not null then will be set to non-zero to indicate that the socket can be written to.
		\returns -1 on error, 0 on timeout, or a positive number otherwise
	*/
	int select(int mstimeout, int *canRead, int *canWrite)
	{
		return XsSocket_select(this, mstimeout, canRead, canWrite);
	}

	/*! \brief \copybrief XsSocket_accept

		\param[in] mstimeout the timeout in milliseconds. Negative timeouts count as infinite; XSSOCKET_INFINITE_TIMEOUT (-1) by default
		\returns a pointer to a newly created XsSocket, NULL on error. The pointer can be deleted.

		\see listen
		\see XsSocket_accept
	*/
	XsSocket* accept(int mstimeout = XSSOCKET_INFINITE_TIMEOUT)
	{
		return XsSocket_accept(this, mstimeout);
	}

	/*! \brief Allocate memory for a socket */
	inline void* operator new(size_t)
	{
		XsSocket *s = XsSocket_allocate();
		if (!s)
			throw std::bad_alloc();
		return s;
	}

	/*! \brief De-allocate the socket */
	inline void operator delete(void* p) noexcept
	{
		XsSocket_free(reinterpret_cast<XsSocket*>(p));
	}

	/*! \brief \copybrief XsSocket_setSocketOption
		\param[in] option The socket option to change
		\param[in] valuePtr Points to the new value of the socket option
		\param[in] valueSize The size of the value \a valuePtr points to
		\returns XRV_OK on success, another XsResultValue otherwise
		\see XsSocket_setSocketOption
	*/
	XsResultValue setSocketOption(XsSocketOption option, void* valuePtr, int valueSize)
	{
		return XsSocket_setSocketOption(this, option, valuePtr, valueSize);
	}

	/*! \brief Sets an socket option of int type
		\param[in] option The socket option to change
		\param[in] value The new value of the socket option
		\returns XRV_OK on success, another XsResultValue otherwise
		\see XsSocket_setSocketOption
	*/
	XsResultValue setSocketOption(XsSocketOption option, int value)
	{
		return XsSocket_setSocketOption(this, option, (void*)&value, sizeof(int));
	}

	/*! \brief Bind to any interface on \a port

		\param[in] port the port to bind to

		\returns XRV_OK on success, another XsResultValue otherwise
		\see bind(XsString, port)
		\see XsSocket_bind
	*/
	XsResultValue bind(uint16_t port)
	{
		return XsSocket_bind(this, NULL, port);
	}

	/*! \brief \copybrief XsSocket_bind

		\param[in] hostname the hostname to bind to
		\param[in] port the port to bind to

		\returns XRV_OK on success, another XsResultValue otherwise
		\see bind(uint16_t)
		\see XsSocket_bind
	*/
	XsResultValue bind(const XsString& hostname, uint16_t port)
	{
		return XsSocket_bind(this, &hostname, port);
	}

	/*! \brief \copybrief XsSocket_listen

		\param[in] maxPending the maximum pending connections

		\returns XRV_OK on success, another XsResultValue when an error occurs
		\see XsSocket_listen
		\see XsSocket_accept
	*/
	XsResultValue listen(int maxPending = 20)
	{
		return XsSocket_listen(this, maxPending);
	}

	/*! \brief \copybrief XsSocket_connect

		\param[in] host the host to connect to
		\param[in] port the port to connect to

		\returns XRV_OK on success, another XsResultValue when an error occurs
		\see XsSocket_connect
	*/
	XsResultValue connect(const XsString& host, uint16_t port)
	{
		return XsSocket_connect(this, &host, port);
	}


	/*! \brief Return true if the socket is usable

		\returns true if the socket can be used for communication, false otherwise
		\see XsSocket_isUsable
	*/
	bool isUsable() const
	{
		return XsSocket_isUsable(this) != 0;
	}

	/*! \brief Get the IP address of the remote host this socket is connected to.

		Only works for connected TCP sockets
		\returns the remote host IP address
	*/
	XsString getRemoteAddress() const
	{
		XsString address;
		XsSocket_getRemoteAddress(this, &address);
		return address;
	}

	/*! \brief Return the system error reported by the last operation
	*/
	int getLastSystemError() const
	{
		return XsSocket_getLastSystemError(this);
	}

	/*! \brief Broadcast data over the socket to the port indicated by \a port

		\param[in] data the data to write
		\param[in] size the size of the data
		\param[in] port the port to send data to

		\returns the number of bytes written, -1 on error, depending on whether the majority of sub broadcasts succeeded or failed.
	*/
	int broadcast(const void* data, XsSize size, uint16_t port)
	{
		return XsSocket_broadcast(this, data, size, port);
	}

	/*! \brief Enable sending and receiving broadcasts on this socket
		\details By default sockets do not receive broadcasts and can't send them. This function can enable this option.
		\param enable Set to 1 to enable broadcasts, 0 to disable them again
		\return 1 if successful, 0 if a failure occurred
	*/
	bool enableBroadcasts(bool enable)
	{
		return 0 != XsSocket_enableBroadcasts(this, enable ? 1 : 0);
	}
private:
#endif
	struct XsSocketPrivate *d;
};

#endif
