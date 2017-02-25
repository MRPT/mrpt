/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef ARSOCKET_H
#define ARSOCKET_H


#ifndef WIN32
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/param.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <stdio.h>
#include <errno.h>
#include <stdarg.h>
#endif


#include <memory.h>
#include <string>
#include "ariaTypedefs.h"
#include "ArMutex.h"

class ArFunctor;


/// socket communication wrapper
/**
   ArSocket is a layer which allows people to use the sockets networking
   interface in an operating system independent manner. All of the standard
   commonly used socket functions are implemented (such as open(), close(),
   connect(), accept(), read(), write(), hostToNetOrder(), netToHostOrder()). It also provides additional useful functions like
   writeString(), readString, setCloseCallback().
   This class also contains the file descriptor which identifies the socket to
   the operating system.

   In Windows, the sockets subsystem needs to be initialized and shutdown
   by the program. So when a program starts it must call Aria::init() and
   call Aria::shutdown() when it exits. (Or, to only initialize the socket
   system, and not do any other global Aria initialization, use ArSocket::init()
   and ArSocket::shutdown().)

   @sa @ref socketServerExample.cpp
   @sa @ref socketClientExample.cpp
*/
class ArSocket
{
public:

  enum Type {UDP, TCP, Unknown};
  enum Error {NoErr, NetFail, ConBadHost, ConNoRoute, ConRefused};

  /// Constructor
  AREXPORT ArSocket();

  /// Constructor which automatically connects to a server as a client
  AREXPORT ArSocket(const char *host, int port, Type type);

  /// Constructor which outomatically opens a server port
  AREXPORT ArSocket(int port, bool doClose, Type type);

  /// Destructor
  AREXPORT ~ArSocket();

  /// Initialize the OS sockets system, if neccesary
  AREXPORT static bool init();

  /// Shutdown the OS sockets system, if neccesary
  AREXPORT static void shutdown();

  AREXPORT static bool ourInitialized;

  /// Copy socket structures
  AREXPORT bool copy(int fd, bool doclose);

  /// Copy socket structures
  /*AREXPORT*/void copy(ArSocket *s)
    {myFD=s->myFD; myDoClose=false; mySin=s->mySin;}

  /// Transfer ownership of a socket
  /** transfer() will transfer ownership to this socket. The input socket
      will no longer close the file descriptor when it is destructed.
  */
  /*AREXPORT*/ void transfer(ArSocket *s)
    {myFD=s->myFD; myDoClose=true; s->myDoClose=false; mySin=s->mySin;
     myType=s->myType; strcpy(myIPString, s->myIPString); }

  /// Connect as a client to a server
  AREXPORT bool connect(const char *host, int port, Type type = TCP);

  /** Open a server port
	  @param openOnIP If given, only check ports open on the interface accociated with this address (Linux only)
  */
  AREXPORT bool open(int port, Type type, const char *openOnIP = NULL);

  /// Simply create a port.
  AREXPORT bool create(Type type);

  /** Find the first valid unused port after @a startPort, and bind the socket to it.
      @param startPort first port to try
	  @param openOnIP If given, only check ports open on the interface accociated with this address (Linux only)
  */
  AREXPORT bool findValidPort(int startPort, const char *openOnIP = NULL);

  /// Connect the socket to the given address
  AREXPORT bool connectTo(const char *host, int port);

  /// Connect the socket to the given address
  AREXPORT bool connectTo(struct sockaddr_in *sin);

  /// Accept a new connection
  AREXPORT bool accept(ArSocket *sock);

  /// Close the socket
  AREXPORT bool close();

  /// Write to the socket
  AREXPORT int write(const void *buff, size_t len);

  /// Read from the socket
  AREXPORT int read(void *buff, size_t len, unsigned int msWait = 0);

  /// Send a message on the socket
  AREXPORT int sendTo(const void *msg, int len);

  /// Send a message on the socket
  AREXPORT int sendTo(const void *msg, int len, struct sockaddr_in *sin);

  /// Receive a message from the socket
  AREXPORT int recvFrom(void *msg, int len, sockaddr_in *sin);

  /// Convert a host string to an address structure
  AREXPORT static bool hostAddr(const char *host, struct in_addr &addr);

  /// Convert an address structure to a host string
  AREXPORT static bool addrHost(struct in_addr &addr, char *host);

  /// Get the localhost address
  AREXPORT static std::string getHostName();

  /// Get the socket name. Stored in ArSocket::mySin
  AREXPORT bool getSockName();

  /// Accessor for the sockaddr
  struct sockaddr_in * sockAddrIn() {return(&mySin);}

  /// Accessor for the in_addr
  struct in_addr * inAddr() {return(&mySin.sin_addr);}

  /// Accessor for the port of the sockaddr
  unsigned short int inPort() {return(mySin.sin_port);}

  /// Convert addr into string numerical address
  AREXPORT static void inToA(struct in_addr *addr, char *buff);

  /// Size of the sockaddr
  static size_t sockAddrLen() {return(sizeof(struct sockaddr_in));}

#ifdef WIN32
  /// Max host name length
  static size_t maxHostNameLen() {return(MAXGETHOSTSTRUCT);}
#else
  /// Max host name length
  static size_t maxHostNameLen() {return(MAXHOSTNAMELEN);}
#endif

  /// Convert an int from host byte order to network byte order
  AREXPORT static unsigned int hostToNetOrder(int i);

  /// Convert an int from network byte order to host byte order
  AREXPORT static unsigned int netToHostOrder(int i);

  /// Set the linger value
  AREXPORT bool setLinger(int time);

  /// Set broadcast value
  AREXPORT bool setBroadcast();

  /// Set the reuse address value
  AREXPORT bool setReuseAddress();

  /// Set socket to nonblocking
  AREXPORT bool setNonBlock();

  /// Change the doClose value
  /*AREXPORT*/ void setDoClose(bool yesno) {myDoClose=yesno;}

  /// Get the file descriptor
  /*AREXPORT*/ int getFD() const {return(myFD);}

  /// Get the protocol type
  /*AREXPORT*/ Type getType() const {return(myType);}

  /// Get the last error string
  /*AREXPORT*/ const std::string & getErrorStr() const {return(myErrorStr);}

  /// Get the last error
  /*AREXPORT*/ Error getError() const {return(myError);}

#ifndef SWIG
  /** @brief Writes a string to the socket (adding end of line characters)
   *  @swigomit
   */
  AREXPORT int writeString(const char *str, ...);
#endif
  /// Same as writeString, but no varargs, wrapper for java
  /*AREXPORT*/ int writeStringPlain(const char *str) { return writeString(str); }
  /// Reads a string from the socket
  AREXPORT char *readString(void);
  /// Sets echoing on the readString calls this socket does
  /*AREXPORT*/ void setEcho(bool echo)
  { myStringAutoEcho = false; myStringEcho = echo; }
  /// Gets if we are echoing on the readString calls this socket does
  /*AREXPORT*/ bool getEcho(void) { return myStringEcho; }
  /// Sets whether we log the writeStrings or not
  /*AREXPORT*/void setLogWriteStrings(bool logWriteStrings)
    { myLogWriteStrings = logWriteStrings; }
  /// Gets whether we log the writeStrings or not
  /*AREXPORT*/ bool getLogWriteStrings(void) { return myLogWriteStrings; }
  /// Gets the ip number as a string
  /*AREXPORT*/ const char *getIPString(void) const { return myIPString; }
  /// Sets the callback for when the socket is closed (nicely or harshly)
  /*AREXPORT*/void setCloseCallback(ArFunctor *functor)
    { myCloseFunctor = functor; }
  /// Sets the callback for when the socket is closed (nicely or harshly)
  /*AREXPORT*/ ArFunctor *getCloseCallback(void) { return myCloseFunctor; }
  /// Gets the number of writes we've done
  long getSends(void) { return mySends; }
  /// Gets the number of bytes we've written
  long getBytesSent(void) { return myBytesSent; }
  /// Gets the number of reads we've done
  long getRecvs(void) { return myRecvs; }
  /// Gets the number of bytes we've read
  long getBytesRecvd(void) { return myBytesRecvd; }
  /// Resets the tracking information on the socket
  void resetTracking(void)
    { mySends = 0; myBytesSent = 0; myRecvs = 0; myBytesRecvd = 0; }

  /// Sets NODELAY option on TCP socket, which can reduce latency for small packet sizes.
  AREXPORT bool setNoDelay(bool flag);

protected:
  /// internal function that sets the ip string from the inAddr
  void setIPString(void);
  /// internal function that echos strings from read string
  void doStringEcho(void);
  // internal crossplatform init (mostly for string reading stuff)
  void internalInit(void);

  Type myType;
  Error myError;
  std::string myErrorStr;
  bool myDoClose;
  int myFD;
  bool myNonBlocking;
  struct sockaddr_in mySin;

  bool myLogWriteStrings;
  ArMutex myReadStringMutex;
  ArMutex myWriteStringMutex;
  bool myStringAutoEcho;
  bool myStringEcho;
  char myStringBuf[1100];
  size_t myStringPos;
  char myStringBufEmpty[1];
  size_t myStringPosLast;
  char myIPString[128];
  bool myStringGotEscapeChars;
  bool myStringGotComplete;
  bool myStringHaveEchoed;

  long mySends;
  long myBytesSent;
  long myRecvs;
  long myBytesRecvd;

  // A functor to call when the socket closes
  ArFunctor *myCloseFunctor;
};


#endif // ARSOCKET_H

