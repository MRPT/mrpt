/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef ARROBOTPACKET_H
#define ARROBOTPACKET_H

#include "ariaTypedefs.h"
#include "ArBasePacket.h"
#include "ariaUtil.h"

/// Represents the packets sent to the robot as well as those received from it
/**
   This class reimplements some of the buf operations since the robot is 
   opposeite endian from intel.  Also has the getID for convenience.  
   
   You can just look at the documentation for the ArBasePacket except for
   the 4 new functions here, verifyCheckSum, getID, print, and calcCheckSum.
 */
class ArRobotPacket: public ArBasePacket
{
public:
  /// Constructor
  AREXPORT ArRobotPacket(unsigned char sync1 = 0xfa, 
			 unsigned char sync2 = 0xfb);
  /// Destructor
  AREXPORT virtual ~ArRobotPacket();

  /// returns true if the checksum matches what it should be
  AREXPORT bool verifyCheckSum(void);

  /// returns the ID of the packet 
  AREXPORT ArTypes::UByte getID(void);

  /// Sets the ID of the packet 
  AREXPORT void setID(ArTypes::UByte id);

  /// returns the checksum, probably used only internally
  AREXPORT ArTypes::Byte2 calcCheckSum(void);
  
  // only call finalizePacket before a send
  AREXPORT virtual void finalizePacket(void);
  
  /// Gets the time the packet was received at
  AREXPORT ArTime getTimeReceived(void);
  /// Sets the time the packet was received at
  AREXPORT void setTimeReceived(ArTime timeReceived);

protected:
  unsigned char mySync1;
  unsigned char mySync2;
  ArTime myTimeReceived;
};

#endif // ARROBOTPACKET_H
