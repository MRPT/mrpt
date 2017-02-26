/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef ARROBOTPACKETRECEIVER_H
#define ARROBOTPACKETRECEIVER_H

#include "ariaTypedefs.h"
#include "ArRobotPacket.h"

class ArDeviceConnection;

/// Given a device connection it receives packets from the robot through it
class ArRobotPacketReceiver
{
public:
  /// Constructor without an already assigned device connection
  AREXPORT ArRobotPacketReceiver(bool allocatePackets = false,
				 unsigned char sync1 = 0xfa, 
				 unsigned char sync2 = 0xfb);
  /// Constructor with assignment of a device connection
  AREXPORT ArRobotPacketReceiver(ArDeviceConnection *deviceConnection, 
				 bool allocatePackets = false,
				 unsigned char sync1 = 0xfa, 
				 unsigned char sync2 = 0xfb);
  /// Destructor
  AREXPORT virtual ~ArRobotPacketReceiver();
  
  /// Receives a packet from the robot if there is one available
  AREXPORT ArRobotPacket *receivePacket(unsigned int msWait = 0);

  /// Sets the device this instance receives packets from
  AREXPORT void setDeviceConnection(ArDeviceConnection *deviceConnection);
  /// Gets the device this instance receives packets from
  AREXPORT ArDeviceConnection *getDeviceConnection(void);
  
  /// Gets whether or not the receiver is allocating packets
  /*AREXPORT*/ bool isAllocatingPackets(void) { return myAllocatePackets; }

protected:
  ArDeviceConnection *myDeviceConn;
  bool myAllocatePackets;
  ArRobotPacket myPacket;
  enum { STATE_SYNC1, STATE_SYNC2, STATE_ACQUIRE_DATA };
  unsigned char mySync1;
  unsigned char mySync2;
};

#endif // ARROBOTPACKETRECEIVER_H
