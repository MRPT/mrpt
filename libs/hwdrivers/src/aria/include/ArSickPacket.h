/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef ARSICKPACKET_H
#define ARSICKPACKET_H

#include "ariaTypedefs.h"
#include "ArBasePacket.h"
#include "ariaUtil.h"

/// Represents the packets sent to the sick as well as those received from it
/**
   This class reimplements some of the buf operations since the robot is 
   little endian. 
   
   You can just look at the documentation for the ArBasePacket except
   for these functions here, setAddress, getAddress, verifyCheckSum,
   print, getID, and calcCheckSum.  
*/

class ArSickPacket: public ArBasePacket
{
public:
  /// Constructor
  AREXPORT ArSickPacket(unsigned char sendingAddress = 0);
  /// Destructor
  AREXPORT virtual ~ArSickPacket();

  /// Sets the address to send this packet to (only use for sending)
  AREXPORT void setSendingAddress(unsigned char address);

  /// Sets the address to send this packet to (only use for sending)
  AREXPORT unsigned char getSendingAddress(void);

  /// Gets the address this packet was sent from (only use for receiving)
  AREXPORT unsigned char getReceivedAddress(void);
  
  /// returns true if the crc matches what it should be
  AREXPORT bool verifyCRC(void);
  
  /// returns the ID of the packet (first byte of data)
  AREXPORT ArTypes::UByte getID(void);

  /// returns the crc, probably used only internally
  AREXPORT ArTypes::Byte2 calcCRC(void);
  
  // only call finalizePacket before a send
  AREXPORT virtual void finalizePacket(void);
  AREXPORT virtual void resetRead(void);
  
  /// Gets the time the packet was received at
  AREXPORT ArTime getTimeReceived(void);
  /// Sets the time the packet was received at
  AREXPORT void setTimeReceived(ArTime timeReceived);

  /// Duplicates the packet
  AREXPORT virtual void duplicatePacket(ArSickPacket *packet);
protected:
  ArTime myTimeReceived;
  unsigned char mySendingAddress;
};

#endif // ARSICKPACKET_H
