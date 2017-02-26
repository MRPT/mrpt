/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef ARSICKPACKETRECEIVER_H
#define ARSICKPACKETRECEIVER_H

#include "ariaTypedefs.h"
#include "ArDeviceConnection.h"
#include "ArSickPacket.h"

/// Given a device connection it receives packets from the sick through it
class ArSickPacketReceiver
{
public:
  /// Constructor without an already assigned device connection
  AREXPORT ArSickPacketReceiver(unsigned char receivingAddress = 0, 
				bool allocatePackets = false,
				bool useBase0Address = false);
  /// Constructor with assignment of a device connection
  AREXPORT ArSickPacketReceiver(ArDeviceConnection *deviceConnection, 
				unsigned char receivingAddress = 0,
				bool allocatePackets = false,
				bool useBase0Address = false);
  /// Destructor
  AREXPORT virtual ~ArSickPacketReceiver();
  
  /// Receives a packet from the robot if there is one available
  AREXPORT ArSickPacket *receivePacket(unsigned int msWait = 0);

  /// Sets the device this instance receives packets from
  AREXPORT void setDeviceConnection(ArDeviceConnection *deviceConnection);
  /// Gets the device this instance receives packets from
  AREXPORT ArDeviceConnection *getDeviceConnection(void);
  
  /// Gets whether or not the receiver is allocating packets
  /*AREXPORT*/ bool isAllocatingPackets(void) { return myAllocatePackets; }

protected:
  ArDeviceConnection *myDeviceConn;
  bool myAllocatePackets;
  ArSickPacket myPacket;
  unsigned char myReceivingAddress;
  bool myUseBase0Address;
  enum { STATE_START, STATE_ADDR, STATE_START_COUNT, STATE_ACQUIRE_DATA };
};

#endif // ARSICKPACKETRECEIVER_H
