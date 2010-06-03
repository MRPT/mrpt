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
