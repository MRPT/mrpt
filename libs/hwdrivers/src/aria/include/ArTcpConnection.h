/*
MobileRobots Advanced Robotics Interface for Applications (ARIA)
Copyright (C) 2004, 2005 ActivMedia Robotics LLC
Copyright (C) 2006, 2007 MobileRobots Inc.

     This program is free software; you can redistribute it and/or modify
     it under the terms of the GNU General Public License as published by
     the Free Software Foundation; either version 2 of the License, or
     (at your option) any later version.

     This program is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
     GNU General Public License for more details.

     You should have received a copy of the GNU General Public License
     along with this program; if not, write to the Free Software
     Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

If you wish to redistribute ARIA under different terms, contact 
MobileRobots for information about a commercial version of ARIA at 
robots@mobilerobots.com or 
MobileRobots Inc, 19 Columbia Drive, Amherst, NH 03031; 800-639-9481
*/

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
