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
#include "ArSickPacket.h"
#include "stdio.h"

AREXPORT ArSickPacket::ArSickPacket(unsigned char sendingAddress) :
  ArBasePacket(2048, 4)
{
  mySendingAddress = sendingAddress;
}

AREXPORT ArSickPacket::~ArSickPacket()
{
}

/** 
   This sets the address for use in sending packets, the address
   is saved, then when a packet is finalizePacketd for sending, the address
   is put into the appropriate spot in the packet.
   @param address the address of the laser to be addressed
*/
AREXPORT void ArSickPacket::setSendingAddress(unsigned char address)
{
  mySendingAddress = address;
}

/** 
   This gets the address for use in sending packets, the address is
   what has been saved, then when a packet is finalizePacketd for sending,
   the address is put into the appropriate spot in the packet. 
   @return the address of the laser to be addressed 
*/
AREXPORT unsigned char ArSickPacket::getSendingAddress(void)
{
  return mySendingAddress;
}

/**
   This gets the address that this packet was received from.  Note that 
   this is only valid if this packet was received from a laser, if you want
   to know where a packet was addressed to use getSendingAdress instead.
   @return the address a packet was received from
*/
AREXPORT unsigned char ArSickPacket::getReceivedAddress(void)
{
  int len = myReadLength;
  unsigned char address;
  
  // toss it into the second byte of the packet
  myReadLength = 1;
  address = bufToUByte();
  myLength = len;
  return address;
}

AREXPORT ArTypes::UByte ArSickPacket::getID(void)
{
 if (myLength >= 5)
    return myBuf[4];
  else
    return 0;
}

AREXPORT void ArSickPacket::resetRead(void)
{
  myReadLength = myHeaderLength + 1;
}

AREXPORT void ArSickPacket::finalizePacket(void)
{
  int len = myLength;
  int chkSum;

  // put in the start of the packet
  myLength = 0;
  // toss in the header 
  uByteToBuf(0x02);
  // now the laser we want to talk to
  uByteToBuf(mySendingAddress);
  // dump in the length
  uByte2ToBuf(len - myHeaderLength);
  myLength = len;

  // that lovely CRC
  chkSum = calcCRC();
  byteToBuf(chkSum & 0xff );
  byteToBuf((chkSum >> 8) & 0xff );

  //printf("Sending ");
  //log();
}

/**
   Copies the given packets buffer into the buffer of this packet, also
   sets this length and readlength to what the given packet has
   @param packet the packet to duplicate
*/
AREXPORT void ArSickPacket::duplicatePacket(ArSickPacket *packet)
{
  myLength = packet->getLength();
  myReadLength = packet->getReadLength();
  myTimeReceived = packet->getTimeReceived();
  mySendingAddress = packet->getSendingAddress();
  memcpy(myBuf, packet->getBuf(), myLength);
  
}

AREXPORT ArTypes::Byte2 ArSickPacket::calcCRC(void)
{
  unsigned short uCrc16;
  unsigned char abData[2];
  unsigned int uLen = myLength;
  unsigned char * commData = (unsigned char *)myBuf;

  uCrc16 = 0;
  abData[0] = 0;
  while (uLen--)
  {
    abData[1] = abData[0];
    abData[0] = *commData++;
    if (uCrc16 & 0x8000)
    {
      uCrc16 = (uCrc16 & 0x7fff) << 1;
      uCrc16 ^= 0x8005;
    }
    else
    {
      uCrc16 <<= 1;
    }
    uCrc16 ^= ((unsigned short) abData[0] | 
	       ((unsigned short)(abData[1]) << 8));
  }
  return uCrc16;
}

AREXPORT bool ArSickPacket::verifyCRC(void) 
{
  int readLen = myReadLength;
  int len = myLength;
  ArTypes::Byte2 chksum;
  unsigned char c1, c2;

  myReadLength = myLength - 2;
  
  if (myReadLength < myHeaderLength)
    return false;

  c1 = bufToByte();
  c2 = bufToByte();
  myReadLength = readLen;
  chksum = (c1 & 0xff) | (c2 << 8);

  myLength = myLength - 2;
  if (chksum == calcCRC()) {
    myLength = len;
    return true;
  } else {
    myLength = len;
    return false;
  }
  
}

AREXPORT ArTime ArSickPacket::getTimeReceived(void)
{
  return myTimeReceived;
}

AREXPORT void ArSickPacket::setTimeReceived(ArTime timeReceived)
{
  myTimeReceived = timeReceived;
}
