/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef ARTCPCONNECTION_H
#define ARTCPCONNECTION_H

#include "ArDeviceConnection.h"
#include <string>

#include "ariaTypedefs.h"
#include "ArSocket.h"

/// For connectiong to a device through a socket
class ArTcpConnection: public ArDeviceConnection
{
 public:
  /// Constructor
  AREXPORT ArTcpConnection();
  /// Destructor also closes connection
  AREXPORT virtual ~ArTcpConnection();

  /// Opens a connection to the given host and port
  AREXPORT int open(const char * host = NULL, int port = 8101);

  AREXPORT void setPort(const char *host = NULL, int port = 8101);
  AREXPORT virtual bool openSimple(void);  
  AREXPORT virtual int getStatus(void);
  AREXPORT virtual bool close(void);
  AREXPORT virtual int read(const char *data, unsigned int size, 
			    unsigned int msWait = 0);
  AREXPORT virtual int write(const char *data, unsigned int size);
  AREXPORT virtual const char * getOpenMessage(int messageNumber);
  AREXPORT virtual ArTime getTimeRead(int index);
  AREXPORT virtual bool isTimeStamping(void);

  /// Gets the name of the host connected to
  AREXPORT std::string getHost(void);
  /// Gets the number of the port connected to
  AREXPORT int getPort(void);

  /// Internal function used by open and openSimple
  AREXPORT int internalOpen(void);

  /// Sets the tcp connection to use this socket instead of its own
  AREXPORT void setSocket(ArSocket *socket);
  /// Gets the socket this tcp connection is using
  AREXPORT ArSocket *getSocket(void);
  /// Sets the status of the device, ONLY use this if you're playing
  /// with setSocket and know what you're doing
  AREXPORT void setStatus(int status);

  enum Open { 
      OPEN_NET_FAIL = 1, ///< Some critical part of the network isn't working
      OPEN_BAD_HOST, ///< Could not find the host
      OPEN_NO_ROUTE, ///< Know where the host is, but can't get to it
      OPEN_CON_REFUSED ///< Got to the host but it didn't allow a connection
  };



protected:
  void buildStrMap(void);
  
  ArStrMap myStrMap;
  bool myOwnSocket;
  ArSocket *mySocket;
  int myStatus;
  
  std::string myHostName;
  int myPortNum;
};

#endif //ARTCPCONNECTION_H
