/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "ArExport.h"
#include "ariaOSDef.h"
#include "ArRobotPacket.h"
#include "stdio.h"

/**
   @param sync1 first byte of the header of this packet, this should be left as
   the default in nearly all cases, ie don't mess with it
   @param sync2 second byte of the header of this packet, this should be left
   as the default in nearly all cases, ie don't mess with it
 */
AREXPORT ArRobotPacket::ArRobotPacket(unsigned char sync1,
				      unsigned char sync2) :
    ArBasePacket(265, 4, NULL, 2)
{
  mySync1 = sync1;
  mySync2 = sync2;
}

AREXPORT ArRobotPacket::~ArRobotPacket()
{
}

AREXPORT ArTypes::UByte ArRobotPacket::getID(void)
{
  if (myLength >= 4)
    return myBuf[3];
  else
    return 0;
}

AREXPORT void ArRobotPacket::setID(ArTypes::UByte id)
{
  myBuf[3] = id;
}

AREXPORT void ArRobotPacket::finalizePacket(void)
{
  int len = myLength;
  int chkSum;

  myLength = 0;
  uByteToBuf(mySync1);
  uByteToBuf(mySync2);
  uByteToBuf(len - getHeaderLength() + 3);
  myLength = len;

  chkSum = calcCheckSum();
  byteToBuf((chkSum >> 8) & 0xff );
  byteToBuf(chkSum & 0xff );
  /* Put this in if you want to see the packets being outputted 
     printf("Output(%3d) ", getID());
     printHex();
  */
  // or put this in if you just want to see the type
  //printf("Output %d\n", getID());
}

AREXPORT ArTypes::Byte2 ArRobotPacket::calcCheckSum(void)
{
  int i;
  unsigned char n;
  int c = 0;

  i = 3;
  n = myBuf[2] - 2;
  while (n > 1) {
    c += ((unsigned char)myBuf[i]<<8) | (unsigned char)myBuf[i+1];
    c = c & 0xffff;
    n -= 2;
    i += 2;
  }
  if (n > 0) 
    c = c ^ (int)((unsigned char) myBuf[i]);
  return c;
}

AREXPORT bool ArRobotPacket::verifyCheckSum(void) 
{
  ArTypes::Byte2 chksum;
  unsigned char c1, c2;

  if (myLength - 2 < myHeaderLength)
    return false;

  c2 = myBuf[myLength - 2];
  c1 = myBuf[myLength - 1];
  chksum = (c1 & 0xff) | (c2 << 8);

  if (chksum == calcCheckSum()) {
    return true;
  } else {
    return false;
  }
  
}

AREXPORT ArTime ArRobotPacket::getTimeReceived(void)
{
  return myTimeReceived;
}

AREXPORT void ArRobotPacket::setTimeReceived(ArTime timeReceived)
{
  myTimeReceived = timeReceived;
}
