/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef ARROBOTPACKETSENDER_H
#define ARROBOTPACKETSENDER_H

#include "ariaTypedefs.h"
#include "ArRobotPacket.h"

class ArDeviceConnection;

/// Given a device connection this sends commands through it to the robot

class ArRobotPacketSender
{
public:
  /// Constructor without an already assigned device connection
  AREXPORT ArRobotPacketSender(unsigned char sync1 = 0xfa,
			       unsigned char sync2 = 0xfb);
  /// Constructor with assignment of a device connection
  AREXPORT ArRobotPacketSender(ArDeviceConnection *deviceConnection,
			       unsigned char sync1 = 0xfa,
			       unsigned char sync2 = 0xfb);
  /// Destructor
  AREXPORT virtual ~ArRobotPacketSender();

  /// Sends a command to the robot with no arguments
  AREXPORT bool com(unsigned char command);
  /// Sends a command to the robot with an int for argument
  AREXPORT bool comInt(unsigned char command, short int argument);
  /// Sends a command to the robot with two bytes for argument
  AREXPORT bool com2Bytes(unsigned char command, char high, char low);
  /// Sends a command to the robot with a length-prefixed string for argument
  AREXPORT bool comStr(unsigned char command, const char *argument);
  /// Sends a command to the robot with a length-prefixed string for argument
  AREXPORT bool comStrN(unsigned char command, const char *str, int size);
  /// Sends a command containing exactly the data in the given buffer as argument
  AREXPORT bool comDataN(unsigned char command, const char *data, int size);
  
  /// Sets the device this instance sends commands to
  AREXPORT void setDeviceConnection(ArDeviceConnection *deviceConnection);
  /// Gets the device this instance sends commands to
  AREXPORT ArDeviceConnection *getDeviceConnection(void);

protected:
  bool connValid(void);
  ArDeviceConnection * myDeviceConn;
  ArRobotPacket myPacket;
  enum { INTARG = 0x3B, NINTARG = 0x1B, STRARG = 0x2B };
};


#endif //ARROBOTPACKETSENDER_H
